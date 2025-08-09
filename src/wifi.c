#include "../include/wifi.h"       
#include "pico/cyw43_arch.h"        
#include <stdio.h>                   
#include "pico/stdio.h"

int status;

void restart_wifi(){
    cyw43_arch_deinit();         
    cyw43_arch_init();
}

void init_wifi(char *text_buffer){
     if (cyw43_arch_init()) {
        sniprintf(text_buffer, 100, "Erro ao iniciar Wi-Fi\n");
        return;
    }
}

bool connect_to_wifi(const char *ssid, const char *password, char *text_buffer) {
    sniprintf(text_buffer, 100, "Conectando...\n");

    // Habilita o modo estação (STA) para se conectar a um ponto de acesso.
    cyw43_arch_enable_sta_mode();

    // Tenta conectar à rede Wi-Fi com um tempo limite de 30 segundos (30000 ms).
    // Utiliza autenticação WPA2 com criptografia AES.
    status = 1;
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, password, CYW43_AUTH_WPA2_AES_PSK, 3000)) {
        sniprintf(text_buffer, 100, "Erro ao conectar\n");
    } else {  
        sniprintf(text_buffer, 100, "Conectado ao Wi-Fi\n");
        return true;      
    }
    return false;
}

bool is_connected(){
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (status == CYW43_LINK_UP) {
        return true;
    }else{
        return false;
    }
}
