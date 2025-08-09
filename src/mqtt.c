#include "lwip/apps/mqtt.h"     
#include "../include/mqtt.h"   
#include "lwipopts.h"            
#include "pico/stdio.h"
#include <stdio.h> 
#include "pico/stdlib.h" 

/* Variável global estática para armazenar a instância do cliente MQTT
 * 'static' limita o escopo deste arquivo */
static mqtt_client_t *client;
bool conected = false;

void meu_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("Payload recebido: %.*s\n", len, data);
}

// Callback chamado quando a inscrição for confirmada
void meu_subscribe_cb(void *arg, err_t result) {
    if (result == ERR_OK) {
        printf("Inscrição realizada com sucesso!\n");
    } else {
        printf("Falha ao se inscrever no tópico.\n");
    }
}

void inscrever_topico(mqtt_client_t *client, const char *topico) {
    mqtt_set_inpub_callback(client, NULL, meu_data_cb, NULL);
    mqtt_sub_unsub(client, topico, 0, meu_subscribe_cb, NULL, 1);
}

/* Função para configurar e iniciar a conexão MQTT
 * Parâmetros:
 *   - client_id: identificador único para este cliente
 *   - broker_ip: endereço IP do broker como string (ex: "192.168.1.1")
 *   - user: nome de usuário para autenticação (pode ser NULL)
 *   - pass: senha para autenticação (pode ser NULL) */
void mqtt_setup(const char *client_id, const char *broker_ip, const char *user, const char *pass, char *text_buffer) {
    if (client != NULL && mqtt_client_is_connected(client)) {
        return;
    }
    ip_addr_t broker_addr;  // Estrutura para armazenar o IP do broker
    snprintf(text_buffer, 100,"Conectando ao broker\n");
    // Converte o IP de string para formato numérico
    if (!ip4addr_aton(broker_ip, &broker_addr)) {
        snprintf(text_buffer, 100, "Erro no IP\n");
        return;
    }

    if (client != NULL) {
        mqtt_disconnect(client);
        mqtt_client_free(client);
        client = NULL;
    }

    // Cria uma nova instância do cliente MQTT
    client = mqtt_client_new();
    if (client == NULL) {
        snprintf(text_buffer, 100,"Falha no cliente\n");
        return;
    }

    // Configura as informações de conexão do cliente
    struct mqtt_connect_client_info_t ci = {
        .client_id = client_id,  // ID do cliente
        .client_user = user,     // Usuário (opcional)
        .client_pass = pass      // Senha (opcional)
    };

    // Inicia a conexão com o broker
    // Parâmetros:
    //   - client: instância do cliente
    //   - &broker_addr: endereço do broker
    //   - 1883: porta padrão MQTT
    //   - mqtt_connection_cb: callback de status (ainda não implementado)
    //   - NULL: argumento opcional para o callback
    //   - &ci: informações de conexão
    mqtt_client_connect(client, &broker_addr, 1883, NULL, &conected, &ci);
}

/* Nova função para configurar e iniciar a conexão MQTT com IP já em ip_addr_t */
void mqtt_setup_ip(const char *client_id, ip_addr_t broker_ip, const char *user, const char *pass, char *text_buffer) {
    if (client != NULL && mqtt_client_is_connected(client)) {
        return;
    }

    if (client != NULL) {
        mqtt_disconnect(client);
        mqtt_client_free(client);
        client = NULL;
    }

    client = mqtt_client_new();
    if (client == NULL) {
        snprintf(text_buffer, 100,"Falha no cliente\n");
        return;
    }

    struct mqtt_connect_client_info_t ci = {
        .client_id = client_id,
        .client_user = user,
        .client_pass = pass
    };

    mqtt_client_connect(client, &broker_ip, 1883, NULL, &conected, &ci);
    snprintf(text_buffer, 100, "Conectando ao broker\n");
}

/* Callback de confirmação de publicação
 * Chamado quando o broker confirma recebimento da mensagem (para QoS > 0)
 * Parâmetros:
 *   - arg: argumento opcional
 *   - result: código de resultado da operação */
static void mqtt_pub_request_cb(void *arg, err_t result) {
    if (result == ERR_OK) {
        printf("Publicação MQTT enviada com sucesso!\n");
    } else {
        printf("Erro ao publicar via MQTT: %d\n", result);
    }
}

/* Função para publicar dados em um tópico MQTT
 * Parâmetros:
 *   - topic: nome do tópico (ex: "sensor/temperatura")
 *   - data: payload da mensagem (bytes)
 *   - len: tamanho do payload */
void mqtt_comm_publish(const char *topic, const uint8_t *data, size_t len) {
    // Envia a mensagem MQTT
    err_t status = mqtt_publish(
        client,              // Instância do cliente
        topic,               // Tópico de publicação
        data,                // Dados a serem enviados
        len,                 // Tamanho dos dados
        0,                   // QoS 0 (nenhuma confirmação)
        1,                   // RETER MENSAGEM ATIVADO
        mqtt_pub_request_cb, // Callback de confirmação
        NULL                 // Argumento para o callback
    );

    if (status != ERR_OK) {
        printf("mqtt_publish falhou ao ser enviada: %d\n", status);
    }
}

bool mqtt_is_connected(){
    if (mqtt_client_is_connected(client)) return true;
    else return false;
}


