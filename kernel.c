void kernel_logic() {
    kernel_pid = getpid(); // O Kernel precisa saber seu próprio PID
    pid_t blocked_queue_d1[NUM_PROCS_APP] = {0}; // Filas de bloqueados de D1
    pid_t blocked_queue_d2[NUM_PROCS_APP] = {0}; // Filas de bloqueados de D2
    int d1_q_size = 0, d2_q_size = 0; // Tamanhos das filas de bloqueados atual
    int current_running_idx = -1;
    int all_processes_terminated = 0;


    //Criando os processos
    for (int i = 0; i < NUM_PROCS_APP; i++) {
        pid_t pid = fork();
        if (pid == 0) {
            close(syscall_pipe[0]);
            close(irq_pipe[0]); 
            application_logic(i, &process_table[i]);
            close(irq_pipe[1]);
            exit(0);
        } else {
            process_table[i].pid = pid;
            process_table[i].state = READY;
            process_table[i].pc = 0;
            process_table[i].d1_access_count = 0;
            process_table[i].d2_access_count = 0;
            ready_queue_processes[total_ready_processes++] = pid;
            printf("[Kernel] Processo de aplicação A%d criado com PID %d e adicionado à fila de prontos.\n", i + 1, pid);
        }
    }
    close(syscall_pipe[1]); // Fecha a extremidade de escrita do pipe de syscalls no kernel
    // Inicialmente, todos os processos são pausados
    for(int i = 0; i < NUM_PROCS_APP; i++) {
        kill(ready_queue_processes[i], SIGSTOP);
    }
    
    current_running_idx = 0;
    process_table[current_running_idx].state = RUNNING;
    kill(process_table[current_running_idx].pid, SIGCONT);
    printf("[Kernel] Escalonando processo A%d (PID %d).\n", current_running_idx + 1, process_table[current_running_idx].pid);
    dequeue_ready(); // Remove o processo que está sendo executado da fila de prontos
    printf("Apos\n");
    for(int i = 0; i < total_ready_processes; i++){
        printf("Fila de prontos: PID %d\n", ready_queue_processes[i]);
    }

    while (1) {

        //PARTE CTRL  C ERRADA
        if (display_status_flag) {
            display_process_status();
            display_status_flag = 0;
            pause();
            printf("\n[Kernel] Retomando a execução...\n");
        }

        InterruptType received_irq;
        int bytes_read = read(irq_pipe[0], &received_irq, sizeof(InterruptType));

        if (bytes_read > 0) {
            printf("[Kernel] Recebeu IRQ%d.\n", received_irq);
            switch (received_irq) {
                case IRQ_TIMESLICE: {
                    if (current_running_idx != -1 && process_table[current_running_idx].state == RUNNING)  {

                        kill(process_table[current_running_idx].pid, SIGSTOP);
                        process_table[current_running_idx].state = READY;
                        enqueue_ready(process_table[current_running_idx].pid); // coloca o processo que acabou de rodar no fim da fila
                        

                        printf("[Kernel] Timeslice! Processo A%d (PID %d) foi interrompido.\n", current_running_idx + 1, process_table[current_running_idx].pid);
                        printf("Apos\n");
                        for(int i = 0; i < total_ready_processes; i++){
                            printf("Fila de prontos: PID %d\n", ready_queue_processes[i]);
                        }
                        //Descobrir a posição do próximo processo pronto na tabela de processos
                        for (int i = 0; i < NUM_PROCS_APP; i++){
                            if (process_table[i].pid == ready_queue_processes[0]){
                                current_running_idx = i;
                                break;
                            }
                        }


                        dequeue_ready(); // remove o processo que será executado da fila de prontos
                        process_table[current_running_idx].state = RUNNING;
                        kill(process_table[current_running_idx].pid, SIGCONT);
                        printf("[Kernel] Contexto trocado para o processo A%d (PID %d).\n", current_running_idx + 1, process_table[current_running_idx].pid);
            
                    }
                    break;
                }


                case IRQ_IO_D1: { 

                    if (d1_q_size > 0){
                        printf("[Kernel] Dispositivo D1 liberado.");

                        for (int i = 0; i < NUM_PROCS_APP; i++){
                            if (process_table[i].pid == blocked_queue_d1[0]) {
                                printf("Desbloqueando processo A%d (PID %d).\n", i + 1, process_table[i].pid);
                                process_table[i].state = READY;
                                process_table[i].d1_access_count++;
                                enqueue_ready(process_table[i].pid);
                                //Retirando o processo da fila de bloqueados
                                for (int j = 1; j <  d1_q_size; j++) {
                                    blocked_queue_d1[j - 1] = blocked_queue_d1[j];
                                }
                                d1_q_size--;
                                break;
                            }
                        }
                    }

                    else{
                        printf("[Kernel] Dispositivo D1 liberado. Nenhum processo estava bloqueado.\n");
                    }
                    break; 
                }
                case IRQ_IO_D2: {
                    if (d2_q_size > 0){
                        printf("[Kernel] Dispositivo D2 liberado.");

                        for (int i = 0; i < NUM_PROCS_APP; i++){
                            if (process_table[i].pid == blocked_queue_d2[0]) {
                                printf("Desbloqueando processo A%d (PID %d).\n", i + 1, process_table[i].pid);
                                process_table[i].state = READY;
                                process_table[i].d2_access_count++;
                                enqueue_ready(process_table[i].pid);
                                //Retirando o processo da fila de bloqueados
                                for (int j = 1; j <  d2_q_size; j++) {
                                    blocked_queue_d1[j - 1] = blocked_queue_d1[j];
                                }
                                d2_q_size--;
                                break;
                            }
                        }
                    }

                    else{
                        printf("[Kernel] Dispositivo D2 liberado. Nenhum processo estava bloqueado.\n");
                    }
                    break; 
                }
            }
        }

        SyscallRequest syscall_req; 
        bytes_read = read(syscall_pipe[0], &syscall_req, sizeof(SyscallRequest));

        if (bytes_read > 0) { // Recebeu uma syscall
            kill(syscall_req.pid, SIGSTOP);
            printf("[Kernel] Recebeu syscall do PID %d (PC=%d).\n", syscall_req.pid, syscall_req.pc);
            dequeue_ready();
            
            for (int i = 0; i < NUM_PROCS_APP; i++) {
                if (process_table[i].pid == syscall_req.pid) { // Encontra o processo que fez a syscall
                    process_table[i].pc = syscall_req.pc;
                    process_table[i].state = BLOCKED;
                    process_table[i].blocked_on_device = syscall_req.device;
                    process_table[i].blocked_on_operation = syscall_req.operation;
                    
                    if (syscall_req.device == DEVICE_D1) blocked_queue_d1[d1_q_size++] = process_table[i].pid;
                    else blocked_queue_d2[d2_q_size++] = process_table[i].pid;
                    
                    printf("[Kernel] Processo A%d (PID %d) bloqueado esperando por %s.\n", i + 1, process_table[i].pid, device_to_str(syscall_req.device));
                    
                    int next_proc_idx = -1; 
                    for (int i = 1; i <= NUM_PROCS_APP; i++) { // Busca o próximo processo pronnto
                        int candidate_idx = (current_running_idx + i) % NUM_PROCS_APP; // busca circular pelo primeiro processo pronto após o que será interrompido
                        if (process_table[candidate_idx].state == READY) {
                            next_proc_idx = candidate_idx;
                            break;
                        }
                    }

                    if (next_proc_idx != -1) { // Encontrou um processo pronto
                        current_running_idx = next_proc_idx;
                        process_table[current_running_idx].state = RUNNING;
                        kill(process_table[current_running_idx].pid, SIGCONT);
                        printf("[Kernel] Contexto trocado para o processo A%d (PID %d).\n", current_running_idx + 1, process_table[current_running_idx].pid);
                    } else {
                        current_running_idx = -1;
                        printf("[Kernel] Nenhum processo na fila de prontos para executar.\n");
                    }
                
                }
            }
        }
        
        for (int i = 0; i < NUM_PROCS_APP; i++) {
            if (process_table[i].state != TERMINATED && process_table[i].pc == MAX_ITERATIONS) {
                process_table[i].state = TERMINATED;
                printf("[Kernel] Processo A%d (PID %d) terminou sua execução.\n", i + 1, process_table[i].pid);
                all_processes_terminated++;
            }
        }

        if (all_processes_terminated == NUM_PROCS_APP) {
            break;
        }
    }
    
    printf("\n[Kernel] Todos os processos de aplicação terminaram. Encerrando o simulador.\n");
    kill(inter_controller_pid, SIGKILL);
    kill(kernel_pid, SIGKILL);
    exit(0);
}