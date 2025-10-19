#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> // uso de valores do tipo "bool": true, false
#include <unistd.h> // fork(), pipe(), read(), write(), close()
#include <signal.h> // signal, kill(), raise()
#include <sys/wait.h> // waitpid()
#include <time.h> // nanosleep(), time()
#include <string.h> // snprintf (função segura de sprintf, que é uma função diferente do printf porque )
#include <errno.h> //  (errno), EAGAIN(erro de recurso não disponível em leitura não bloqueante), EWOULDBLOCK (erro similar)
#include <fcntl.h> // fcntl(), F_GETFL, F_SETFL, O_NONBLOCK (para pipes não bloqueantes)
#include <sys/select.h> // select()

// Configurações
#define NUM_PROCS_APP 5 
#define MAX_ITERATIONS 20 // máximo de iterações por App antes de terminar
#define TIMESLICE_MS 500 // em milissegundos



// Probabilidades 
#define PROB_SYSCALL 10 // a cada iteração, chance de fazer syscall por parte de um procesos
#define P1_PROB 10 // probabilidade de IRQ1 (dispositivo 1) a cada timeslice
#define P2_PROB 5  // probabilidade de IRQ2 (dispositivo 2) a cada timeslice



// Tipos e enums
typedef enum {
    IRQ_TIMESLICE = 0, // IRQ0
    IRQ_IO_D1 = 1, // IRQ1
    IRQ_IO_D2 = 2  // IRQ2
} InterruptType;

typedef enum {
    READY,
    RUNNING,
    BLOCKED,
    TERMINATED
} ProcessState;

typedef enum { 
    DEVICE_D1 = 0, 
    DEVICE_D2 = 1 
} Device;
typedef enum { 
    OP_READ = 0, 
    OP_WRITE = 1, 
    OP_EXEC = 2 
} Operation;




// Mensagens InterController -> Kernel e Apps -> Kernel
typedef struct {
    int type; // qual foi o InterruptType enviado pelo InterController
} IRQMsg;

typedef enum { 
    APP_SYSCALL = 1, 
    APP_TERMINATED = 2, 
    APP_PROGRESS = 3 
} AppMsgType; // tipos de mensagem de que os application process enviam para o Kernel

typedef struct {
    int type;  // AppMsgType
    pid_t pid; // remetente
    int device; // Device (se for syscall)
    int op; // Operation (se for syscall)
} AppMsg;




// PCBs e Tabelas
typedef struct {
    pid_t pid;
    char  name[8];
    ProcessState state;
    int pc; // program counter do processo 

    // bloqueio
    int blocked_dev; // -1 se não bloqueado
    int blocked_op;  // válido se bloqueado

    // contadores por operação
    int count_read;
    int count_write;
    int count_exec;

    bool alive; // se o filho ainda existe (não reaproveitado)
} PCB;

// --------------- Filas simples (array circular) ---------------
#define QMAX 128

typedef struct {
    pid_t data[QMAX];
    int head, tail, size;
} PIDQueue;

static void q_init(PIDQueue *q){ q->head = q->tail = q->size = 0; }
static bool q_empty(PIDQueue *q){ return q->size == 0; }
static bool q_full (PIDQueue *q){ return q->size == QMAX; }
static bool q_push (PIDQueue *q, pid_t v){
    if(q_full(q)) return false;
    q->data[q->tail] = v;
    q->tail = (q->tail + 1) % QMAX;
    q->size++;
    return true;
}
static pid_t q_pop(PIDQueue *q){
    if(q_empty(q)) return -1;
    pid_t v = q->data[q->head];
    q->head = (q->head + 1) % QMAX;
    q->size--;
    return v;
}

// --------------- Globais (apenas no Kernel) ---------------
static PCB pcb[NUM_PROCS_APP];
static PIDQueue ready_q;
static PIDQueue blocked_d1_q;
static PIDQueue blocked_d2_q;

static int irq_pipe[2] = {-1,-1}; // [0]=read no Kernel; [1]=write no InterController
static int sys_pipe[2] = {-1,-1}; // [0]=read no Kernel; [1]=write nos Apps (compartilhado)

static int app_index_from_pid(pid_t p){
    for(int i=0;i<NUM_PROCS_APP;i++) if(pcb[i].pid == p) return i;
    return -1;
}

static volatile sig_atomic_t got_sigint = 0;
static void sigint_handler(int sig){ (void)sig; got_sigint = 1; }
static void sigusr1_handler(int sig){ (void)sig; /* apenas acorda pause/select */ }

// Pretty strings
static const char* state_str(ProcessState s){
    switch(s){
        case READY: return "READY";
        case RUNNING: return "RUNNING";
        case BLOCKED: return "BLOCKED";
        case TERMINATED: return "TERMINATED";
        default: return "?";
    }
}
static const char* dev_str(int d){ return d==0?"D1":"D2"; }
static const char* op_str(int o){
    switch(o){ case OP_READ: return "READ"; case OP_WRITE: return "WRITE"; case OP_EXEC: return "EXEC"; default: return "?"; }
}

// --------------- Util ---------------
static int set_nonblock(int fd){
    int flags = fcntl(fd, F_GETFL, 0);
    if(flags < 0) return -1;
    if(fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return -1;
    return 0;
}

static void print_status_table(void){
    fprintf(stderr, "\n===== STATUS (Kernel %d) =====\n", getpid()); // stderr é o c
    fprintf(stderr, " PID     | Name |   State   |  PC  | Blocked | Op   | R  W  X \n");
    fprintf(stderr, "---------+------+-----------+------+---------+------+---------\n");
    for(int i=0;i<NUM_PROCS_APP;i++){
        PCB *p = &pcb[i];
        fprintf(stderr, " %-7d | %-4s | %-9s | %-4d | ",
            p->pid, p->name, state_str(p->state), p->pc);
        if(p->state == BLOCKED){
            fprintf(stderr, "%-7s | %-4s | ", dev_str(p->blocked_dev), op_str(p->blocked_op));
        } else {
            fprintf(stderr, "%-7s | %-4s | ", "-", "-");
        }
        fprintf(stderr, "%-2d %-2d %-2d\n", p->count_read, p->count_write, p->count_exec);
    }
    fprintf(stderr, " ReadyQ size=%d | D1Q=%d | D2Q=%d\n", ready_q.size, blocked_d1_q.size, blocked_d2_q.size);
    fprintf(stderr, "================================\n\n");
}

// --------------- Escalonamento ---------------
pid_t current_pid = -1; // PID do processo atualmente em RUNNING começa como -1

// Enfileira o processo antigo (se aplicável) e troca para next_pid
void switch_to(pid_t next_pid){
    if(next_pid <= 0) return; // nada para rodar

    // Pare o atual e re-enfileire se ainda estiver RUNNING
    if(current_pid > 0){
        int idx = app_index_from_pid(current_pid);
        if(idx >= 0 && pcb[idx].state == RUNNING){
            // Preempt: pare o atual
            kill(current_pid, SIGSTOP);
            // Marca como READY e volta para a fila
            pcb[idx].state = READY;
            q_push(&ready_q, current_pid);
            fprintf(stdout, "[Kernel] Troca -> %s (preempt %d)\n", pcb[app_index_from_pid(next_pid)].name, current_pid);
            fflush(stdout);
        }
    }

    // Inicia/continua o próximo
    int nidx = app_index_from_pid(next_pid);
    if(nidx >= 0 && pcb[nidx].state == READY){
        pcb[nidx].state = RUNNING;
        current_pid = next_pid;
        kill(next_pid, SIGCONT);
        fprintf(stdout, "[Kernel] Executando %s\n", pcb[nidx].name);
        fflush(stdout); // força saída imediata com flush
    }
}

// --------------- InterController ---------------
static void intercontroller_process(void){
    close(irq_pipe[0]); // fecha leitura
    // Fechar sys_pipe no IC
    close(sys_pipe[0]); close(sys_pipe[1]); // fecha os pipes de syscall, já que nao usa

    // semente aleatória para random generation
    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = TIMESLICE_MS * 1000000L;

    for(;;){
        // IRQ0: timeslice
        IRQMsg m0 = { .type = IRQ_TIMESLICE };
        if(write(irq_pipe[1], &m0, sizeof(m0)) < 0){
            // Kernel pode ter morrido; encerra
            _exit(0);
        }

        // Gera IRQ1/IRQ2 de forma probabilística (pós timeslice)
        int r = rand()%100;
        if(r < P1_PROB){
            IRQMsg m1 = { .type = IRQ_IO_D1 };
            (void)write(irq_pipe[1], &m1, sizeof(m1));
        }
        r = rand()%100;
        if(r < P2_PROB){
            IRQMsg m2 = { .type = IRQ_IO_D2 };
            (void)write(irq_pipe[1], &m2, sizeof(m2));
        }

        // espera próximo tick
        nanosleep(&ts, NULL);
    }
}

// --------------- App ---------------
static void app_process(int app_no){
    // app_no em 0..NUM_PROCS_APP-1
    // Fechar descritores que não usa
    close(irq_pipe[0]); close(irq_pipe[1]); // apps não usam IRQ pipe
    close(sys_pipe[0]); // app só escreve

    // semente aleatória por processo
    srand((unsigned)time(NULL) ^ (unsigned)getpid() ^ (unsigned)(app_no*7919));

    int pc = 0;
    while(pc < MAX_ITERATIONS){
        // Simula "trabalho": um pequeno busy + sleep
        //for(volatile int k=0;k<1000000;k++); // busy breve
        sleep(1);

        // Chance de syscall
        int r = rand()%100;
        if(r < PROB_SYSCALL){
            // escolhe device/op
            Device d = (rand()%2==0) ? DEVICE_D1 : DEVICE_D2;
            int opraw = rand()%3;
            Operation op = (Operation)opraw;

            AppMsg msg = {.type = APP_SYSCALL, .pid = getpid(), .device = (int)d, .op = (int)op};
            (void)write(sys_pipe[1], &msg, sizeof(msg));

            // "Trapa" de syscall: para a si mesmo. Kernel decidirá o que fazer.
            raise(SIGSTOP);
            // Quando Kernel desbloquear e nos agendar de novo, voltamos daqui.
        } else {
            // avança PC
            pc++;
            AppMsg progress = {.type = APP_PROGRESS, .pid = getpid(), .device = -1, .op = pc};
            (void)write(sys_pipe[1], &progress, sizeof(progress));
        }
    }

    // terminou
    AppMsg done = {.type = APP_TERMINATED, .pid = getpid(), .device = -1, .op = -1};
    (void)write(sys_pipe[1], &done, sizeof(done));
    exit(0);
}

// --------------- Kernel (Main) ---------------
int main(void){
    // Instala sinais para status/pause/retomar
    signal(SIGINT,  sigint_handler); // seta o handler de ctrl c
    signal(SIGUSR1, sigusr1_handler); // seta o handler de SIGUSR1
    signal(SIGCONT, sigusr1_handler);

    if(pipe(irq_pipe) < 0){ perror("pipe irq"); return 1; }
    if(pipe(sys_pipe) < 0){ perror("pipe sys"); return 1; }

    // Kernel (pai) vai ler de irq_pipe[0] e sys_pipe[0]
    // Colocar leitura como não bloqueante ajuda a não travar impressão
    set_nonblock(irq_pipe[0]);
    set_nonblock(sys_pipe[0]);

    // Cria InterController
    pid_t ic_pid = fork();
    if(ic_pid < 0){ perror("fork IC"); return 1; }
    if(ic_pid == 0){
        intercontroller_process();
        return 0;
    }

    // Fecha escrita de IRQ no Kernel; quem escreve é só o IC
    close(irq_pipe[1]);

    // Cria Apps
    q_init(&ready_q); // 
    q_init(&blocked_d1_q);
    q_init(&blocked_d2_q);

    for(int i=0;i<NUM_PROCS_APP;i++){
        snprintf(pcb[i].name, sizeof(pcb[i].name), "A%d", i+1);
        pcb[i].state = READY;
        pcb[i].pc = 0;
        pcb[i].blocked_dev = -1;
        pcb[i].blocked_op = -1;
        pcb[i].count_read = pcb[i].count_write = pcb[i].count_exec = 0;
        pcb[i].alive = false;

        pid_t p = fork();
        if(p < 0){
            perror("fork app");
            return 1;
        }
        if(p == 0){
            // No filho App: pare inicialmente e aguarde ser agendado
            // Para isso: mande SIGSTOP a si mesmo, Kernel fará SIGCONT quando escalar
            raise(SIGSTOP);
            app_process(i);
            return 0;
        }
        // No Kernel: registra PCB e enfileira
        pcb[i].pid = p;
        pcb[i].alive = true;
        q_push(&ready_q, p);
        fprintf(stdout, "[Kernel] %s PID=%d pronto\n", pcb[i].name, p);
        fflush(stdout);
    }

    // Kernel não escreve no sys_pipe; só lê
    close(sys_pipe[1]);

    // Começa executando o primeiro pronto
    if(!q_empty(&ready_q)){
        pid_t first = q_pop(&ready_q);
        // marca como RUNNING e SIGCONT dentro de switch_to
        // Mas current_pid ainda é -1, então switch_to tratará apenas start
        // Primeiro, precisamos colocar o selecionado como READY para switch_to promovê-lo a RUNNING
        int fidx = app_index_from_pid(first);
        if(fidx >= 0) pcb[fidx].state = READY;
        switch_to(first);
    }

    // Loop principal do Kernel
    int apps_terminated = 0;
    fd_set rds;
    int nfds = (irq_pipe[0] > sys_pipe[0] ? irq_pipe[0] : sys_pipe[0]) + 1;

    while(1){
        if(apps_terminated >= NUM_PROCS_APP){
            fprintf(stdout, "[Kernel] Todos os apps terminaram.\n");
            fflush(stdout);
            // Encerra IC e sai
            kill(ic_pid, SIGTERM);
            waitpid(ic_pid, NULL, 0);
            break;
        }

        if(got_sigint){
            got_sigint = 0;
            print_status_table();
            // pausa até receber SIGCONT/SIGUSR1
            pause();
        }

        FD_ZERO(&rds);
        FD_SET(irq_pipe[0], &rds);
        FD_SET(sys_pipe[0], &rds);

        // Espera algo chegar (IRQ0/1/2 ou SYSCALL/TERM)
        int rv = select(nfds, &rds, NULL, NULL, NULL);
        if(rv < 0){
            if(errno == EINTR) continue;
            perror("select");
            break;
        }

        // 1) Mensagens de Apps (syscalls / terminated)
        if(FD_ISSET(sys_pipe[0], &rds)){
            for(;;){
                AppMsg am;
                ssize_t n = read(sys_pipe[0], &am, sizeof(am));
                if(n < 0){
                    if(errno==EAGAIN || errno==EWOULDBLOCK) break;
                    perror("read sys");
                    break;
                }
                if(n == 0) break;

                if(am.type == APP_SYSCALL){
                    int idx = app_index_from_pid(am.pid);
                    if(idx >= 0 && pcb[idx].state != TERMINATED){
                        // marca bloqueado e contabiliza
                        pcb[idx].state = BLOCKED;
                        pcb[idx].blocked_dev = am.device;
                        pcb[idx].blocked_op  = am.op;
                        if(am.op == OP_READ) pcb[idx].count_read++;
                        else if(am.op == OP_WRITE) pcb[idx].count_write++;
                        else if(am.op == OP_EXEC) pcb[idx].count_exec++;

                        // Remove de running se era o atual
                        if(current_pid == am.pid){
                            current_pid = -1;
                        }

                        // coloca na fila do dispositivo
                        if(am.device == DEVICE_D1) q_push(&blocked_d1_q, am.pid);
                        else q_push(&blocked_d2_q, am.pid);

                        fprintf(stdout, "[Kernel] %s fez SYSCALL %s em %s, agora BLOQUEADO\n",
                                pcb[idx].name, op_str(am.op), dev_str(am.device));
                        fflush(stdout);

                        // Escalone imediatamente outro se houver
                        if(!q_empty(&ready_q)){
                            pid_t next = q_pop(&ready_q);
                            int nidx = app_index_from_pid(next);
                            if(nidx >= 0 && pcb[nidx].state == READY){
                                switch_to(next);
                            }
                        }
                    }
                } else if(am.type == APP_TERMINATED){
                    int idx = app_index_from_pid(am.pid);
                    if(idx >= 0 && pcb[idx].state != TERMINATED){
                        pcb[idx].state = TERMINATED;
                        pcb[idx].alive = false;
                        if(current_pid == am.pid) current_pid = -1;
                        apps_terminated++;
                        fprintf(stdout, "[Kernel] %s terminou.\n", pcb[idx].name);
                        fflush(stdout);
                        // escalar o próximo
                        if(!q_empty(&ready_q)){
                            pid_t next = q_pop(&ready_q);
                            int nidx = app_index_from_pid(next);
                            if(nidx >= 0 && pcb[nidx].state == READY){
                                switch_to(next);
                            }
                        }
                    }
                } else if (am.type == APP_PROGRESS) {
                    int idx = app_index_from_pid(am.pid);
                    if (idx >= 0 && pcb[idx].state != TERMINATED) {
                        pcb[idx].pc = am.op; // atualiza o PC
                    }
                }
            }
        }

        // 2) Mensagens de IRQ (IRQ0/1/2)
        if(FD_ISSET(irq_pipe[0], &rds)){ // se houve IRQ (a função fd_isset indica qual foi o canal)
            while(1){
                IRQMsg im;
                ssize_t n = read(irq_pipe[0], &im, sizeof(im));
                if(n < 0){
                    if(errno==EAGAIN || errno==EWOULDBLOCK) break;
                    perror("read irq");
                    break;
                }
                if(n == 0) break;

                if(im.type == IRQ_TIMESLICE){
                    // Round-Robin: pegue próximo pronto e troque
                    if(!q_empty(&ready_q)){
                        pid_t next = q_pop(&ready_q);
                        int nidx = app_index_from_pid(next);
                        if(nidx >= 0){
                            // Garante que next esteja marcado READY
                            if(pcb[nidx].state == READY){
                                switch_to(next);
                            }
                        }
                    } else {
                        // nada pronto; não faz nada (pode estar bloqueado ou ninguém vivo)
                    }
                } else if(im.type == IRQ_IO_D1 || im.type == IRQ_IO_D2){
                    PIDQueue *bq = (im.type == IRQ_IO_D1) ? &blocked_d1_q : &blocked_d2_q;
                    if(!q_empty(bq)){
                        pid_t unb = q_pop(bq);
                        int uidx = app_index_from_pid(unb);
                        if(uidx >= 0 && pcb[uidx].state == BLOCKED){
                            pcb[uidx].state = READY;
                            pcb[uidx].blocked_dev = -1;
                            pcb[uidx].blocked_op  = -1;
                            q_push(&ready_q, unb);
                            fprintf(stdout, "[Kernel] IRQ %s: desbloqueou %s\n",
                                    (im.type==IRQ_IO_D1?"D1":"D2"), pcb[uidx].name);
                            fflush(stdout);
                        }
                    }
                    // nada mais; quem executa será escolhido no próximo IRQ0
                }
            }
        }

        // reap colhe crianças zumbis (apps que já mandaram TERMINATED)
        while(1){
            int status;
            pid_t w = waitpid(-1, &status, WNOHANG);
            if(w <= 0) 
                break;
            // encontrou um filho terminado
        }
    }

    // encerra qualquer resto
    for(int i=0;i<NUM_PROCS_APP;i++){
        if(pcb[i].alive){
            kill(pcb[i].pid, SIGTERM);
            waitpid(pcb[i].pid, NULL, 0);
        }
    }
    return 0;
}