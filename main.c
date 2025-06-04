/******************************************************************************
 *  SISTEMA SASI – Monitor de Temperatura + Previsão (Regressão & Holt) + MQTT
 *  Revisão: 05/06/2025
 *
 *  Publica:
 *      /Temperatura_MQTT _Pico/temperature                (atual)
 *      /Temperatura_MQTT _Pico/temperature_predicted      (regressão linear)
 *      /Temperatura_MQTT _Pico/temperature_predicted_holt (Holt)
 *      /Temperatura_MQTT _Pico/status                     (Normal/Alerta…)
 *
 *  Demais recursos: display OLED, LEDs, matriz 8×8, buzzer, FreeRTOS, Wi-Fi
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <limits.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "pico/cyw43_arch.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"

#include "ssd1306.h"
#include "ds18b20.h"
#include "matriz_led.h"

/*============================================================================
 *  CONFIGURAÇÃO DE REDE  ↴  AJUSTE AQUI
 *===========================================================================*/
#define WIFI_SSID       ""    // Leave blank for security
#define WIFI_PASSWORD   "" // Leave blank for security
#define MQTT_SERVER     ""    // Leave blank for security
#define MQTT_USERNAME   ""     // Leave blank for security
#define MQTT_PASSWORD   "" // Leave blank for security

/*============================================================================
 *  HARDWARE (mesmo do projeto original)
 *===========================================================================*/
#define PINO_SDA_I2C        14
#define PINO_SCL_I2C        15
#define PINO_ADC            26
#define PINO_BOTAO_A        5
#define PINO_BOTAO_B        6
#define PINO_DS18B20        16
#define PINO_LED_VERDE      11
#define PINO_LED_VERMELHO   13
#define PINO_BUZZER         10

/*============================================================================
 *  LÓGICA DE APLICAÇÃO
 *===========================================================================*/
#define TAMANHO_HISTORICO_TEMP      30
#define INTERVALO_PREVISAO_SEGUNDOS 300
#define INTERVALO_LEITURA_SEGUNDOS  5

#define DEBOUNCE_JOYSTICK_MS        300
#define DEBOUNCE_BOTAO_MS           50
#define TIMEOUT_ATUALIZACAO_DISPLAY_MS 100
#define PISCAR_INTERVALO_MS         500

/* ---------- Holt ---------- */
#define ALPHA_HOLT 0.3f   /* suavização nível   */
#define BETA_HOLT  0.1f   /* suavização tendência */

/*============================================================================
 *  MQTT
 *===========================================================================*/
#define MQTT_TOPIC_BASE             "/Temperatura_MQTT _Pico"
#define MQTT_KEEP_ALIVE_S           60
#define TEMP_PUBLISH_INTERVAL_S     10
#define MQTT_SUBSCRIBE_QOS          1
#define MQTT_PUBLISH_QOS            1
#define MQTT_PUBLISH_RETAIN         0
#define MQTT_WILL_TOPIC             "/online"
#define MQTT_WILL_MSG               "0"
#define MQTT_WILL_QOS               1
#define MQTT_DEVICE_NAME            "pico"
#define MQTT_TOPIC_LEN              100

/*============================================================================
 *  ESTRUTURAS
 *===========================================================================*/
typedef struct {
    float temperatura;
    TickType_t marca_tempo;
} DadosTemperatura_t;

typedef struct {
    float previsao_linear;
    float previsao_holt;
} ResultadosPrevisao_t;

typedef enum {
    COMANDO_PROXIMA_TELA,
    COMANDO_TELA_ANTERIOR,
    COMANDO_AJUSTAR_URGENCIA_SUBIR,
    COMANDO_AJUSTAR_URGENCIA_DESCER
} TipoComando_t;

typedef struct { TipoComando_t tipo; int valor; } ComandoUsuario_t;

typedef struct {
    int   temperatura_urgencia;
    float temperatura_atual;
    float temperatura_prevista;       /* regressão linear  */
    float temperatura_prevista_holt;  /* Suavização Holt   */
    int   tela_atual;
    bool  configuracao_concluida;
} EstadoSistema_t;

typedef struct {
    float historico_temperatura[TAMANHO_HISTORICO_TEMP];
    float historico_tempo[TAMANHO_HISTORICO_TEMP];
    int   indice_historico;
    bool  historico_preenchido;
} PrevisaoTemperatura_t;

/* ---------- MQTT ---------- */
typedef struct {
    mqtt_client_t *inst;
    struct mqtt_connect_client_info_t info;
    ip_addr_t server_addr;
    char topic[MQTT_TOPIC_LEN];
    char data[64];
    bool conectado;
} MQTT_STATE_t;

/*============================================================================
 *  VARIÁVEIS GLOBAIS
 *===========================================================================*/
static EstadoSistema_t       estado_sistema = { .temperatura_urgencia = 30 };
static PrevisaoTemperatura_t previsao       = {0};
static ssd1306_t             display;

static SemaphoreHandle_t mutex_estado, mutex_display;
static QueueHandle_t     q_temp, q_prev, q_cmd;

static uint slice_buzzer, channel_buzzer;

static MQTT_STATE_t mqtt_state;

/*============================================================================
 *  MACROS / HELPERS
 *===========================================================================*/
static inline const char *topic_full(const char *sufixo){
    static char buf[MQTT_TOPIC_LEN];
    snprintf(buf,sizeof(buf), MQTT_TOPIC_BASE "%s", sufixo);
    return buf;
}

static void ler_estado(EstadoSistema_t *dst){
    if (xSemaphoreTake(mutex_estado, pdMS_TO_TICKS(5))){
        *dst = estado_sistema;
        xSemaphoreGive(mutex_estado);
    } else memset(dst,0,sizeof(*dst));
}

static const char *situacao(float t,float tprev,int urg){
    float d = urg-tprev;
    if (t>urg)      return "Grave";
    if (d>5.0f)     return "Normal";
    if (d>=0.0f)    return "atencao";
    return "Alerta";
}

/*============================================================================
 *  PREVISÃO - Regressão linear (já havia) + Holt
 *===========================================================================*/
static bool regressao(float *x,float *y,int n,float *m,float *b){
    if (n<2){*m=0;*b=(n?y[0]:0);return false;}
    float sx=0,sy=0,sxy=0,sx2=0;
    for(int i=0;i<n;i++){sx+=x[i];sy+=y[i];sxy+=x[i]*y[i];sx2+=x[i]*x[i];}
    float denom=n*sx2-sx*sx;
    if(fabsf(denom)<1e-6){*m=0;*b=sy/n;return false;}
    *m=(n*sxy-sx*sy)/denom; *b=(sy-*m*sx)/n; return true;
}

static float previsao_linear(float tempo_atual){
    float m,b;
    int n=previsao.historico_preenchido?TAMANHO_HISTORICO_TEMP:previsao.indice_historico;
    if(n>=2 && regressao(previsao.historico_tempo,previsao.historico_temperatura,n,&m,&b)){
        return m*(tempo_atual+INTERVALO_PREVISAO_SEGUNDOS)+b;
    }
    return estado_sistema.temperatura_atual;
}

/*============================================================================
 *  BEEP
 *===========================================================================*/
static void beep(int ms,int rep,int f){
    uint32_t wrap=(1000000/f)-1;
    pwm_set_wrap(slice_buzzer,wrap);
    pwm_set_chan_level(slice_buzzer,channel_buzzer,wrap/2);
    for(int i=0;i<=rep;i++){
        pwm_set_enabled(slice_buzzer,true);
        vTaskDelay(pdMS_TO_TICKS(ms));
        pwm_set_enabled(slice_buzzer,false);
        if(i<rep)vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*============================================================================
 *  INDICADORES VISUAIS (LEDs, Matriz, buzzer)
 *===========================================================================*/
static TickType_t ultimo_piscar=0; static bool visivel=true;
static void indicadores(const char *sit,bool conf){
    if(!conf){gpio_put(PINO_LED_VERDE,0);gpio_put(PINO_LED_VERMELHO,0);
              matriz_clear(); pwm_set_enabled(slice_buzzer,false); return;}
    TickType_t now=xTaskGetTickCount();
    if(now-ultimo_piscar>pdMS_TO_TICKS(PISCAR_INTERVALO_MS)){visivel=!visivel;ultimo_piscar=now;}
    if(!strcmp(sit,"Normal")){
        gpio_put(PINO_LED_VERDE,1);gpio_put(PINO_LED_VERMELHO,0);
        matriz_draw_pattern(PAD_OK,COR_VERDE); pwm_set_enabled(slice_buzzer,false);
    }else if(!strcmp(sit,"atencao")){
        gpio_put(PINO_LED_VERDE,1);gpio_put(PINO_LED_VERMELHO,1);
        if(visivel){matriz_draw_pattern(PAD_EXC,COR_AMARELO);beep(150,0,1500);}else matriz_clear();
    }else if(!strcmp(sit,"Alerta")){
        gpio_put(PINO_LED_VERDE,0);gpio_put(PINO_LED_VERMELHO,1);
        if(visivel){matriz_draw_pattern(PAD_X,COR_VERMELHO);beep(100,1,2000);}else matriz_clear();
    }else{ /* Grave */
        gpio_put(PINO_LED_VERDE,0);gpio_put(PINO_LED_VERMELHO,visivel);
        if(visivel){matriz_draw_pattern(PAD_X,COR_VERMELHO);beep(80,2,2500);}else matriz_clear();
    }
}

/*============================================================================
 *  DISPLAY OLED
 *===========================================================================*/
static void tela_cfg(int urg){
    ssd1306_draw_string(&display,"Temperatura",20,0,false);
    ssd1306_draw_string(&display,"Urgencia",32,16,false);
    char txt[12]; snprintf(txt,sizeof(txt),"%d C",urg);
    ssd1306_draw_string(&display,txt,(128-strlen(txt)*6)/2,40,false);
}
static void tela_res(float t,float prev,float prevH,int urg,const char *sit){
    char buf[30];
    snprintf(buf,sizeof(buf),"Temp urg: %d C",urg);            ssd1306_draw_string(&display,buf,0,0,false);
    snprintf(buf,sizeof(buf),"Atual: %.1fC",t);                ssd1306_draw_string(&display,buf,0,14,false);
    snprintf(buf,sizeof(buf),"Prev lin: %.1fC",prev);          ssd1306_draw_string(&display,buf,0,28,false);
    snprintf(buf,sizeof(buf),"Prev Holt: %.1fC",prevH);        ssd1306_draw_string(&display,buf,0,42,false);
    snprintf(buf,sizeof(buf),"Situacao: %s",sit);              ssd1306_draw_string(&display,buf,0,56,false);
}

/*============================================================================
 *  TASK: LEITURA TEMPERATURA, PREVISÕES
 *===========================================================================*/
static void tarefa_temp(void *p){
    (void)p;
    float nivel=0,trend=0; bool first=true,init_holt=false;
    const int passos_adiantados = INTERVALO_PREVISAO_SEGUNDOS / INTERVALO_LEITURA_SEGUNDOS;
    TickType_t inicio=xTaskGetTickCount();
    while(1){
        float t=ds18b20_get_temperature();
        if(first){nivel=t;trend=0;first=false;}                /* filtro inicial */
        /* filtro exponencial simples p/ atenuar ruído */
        static float t_filt=0; static bool ff=true;
        t_filt = ff ? (ff=false,t) : t_filt*0.8f+t*0.2f;

        if(t>-20&&t<80){
            /* --- histórico regressão --- */
            float tempo=(xTaskGetTickCount()-inicio)*portTICK_PERIOD_MS/1000.0f;
            previsao.historico_temperatura[previsao.indice_historico]=t_filt;
            previsao.historico_tempo[previsao.indice_historico]=tempo;
            previsao.indice_historico=(previsao.indice_historico+1)%TAMANHO_HISTORICO_TEMP;
            if(!previsao.historico_preenchido && previsao.indice_historico==0)
                previsao.historico_preenchido=true;

            /* --- Holt --- */
            if(!init_holt){nivel=t_filt;trend=0;init_holt=true;}
            float prev_nivel=nivel;
            nivel = ALPHA_HOLT*t_filt + (1-ALPHA_HOLT)*(nivel+trend);
            trend = BETA_HOLT*(nivel-prev_nivel) + (1-BETA_HOLT)*trend;
            float holt_prev = nivel + trend*passos_adiantados;

            /* --- regressão linear --- */
            float lin_prev = previsao_linear(tempo);

            /* envia às filas */
            DadosTemperatura_t d={.temperatura=t_filt,.marca_tempo=xTaskGetTickCount()};
            xQueueSend(q_temp,&d,0);
            ResultadosPrevisao_t r={.previsao_linear=lin_prev,.previsao_holt=holt_prev};
            xQueueSend(q_prev,&r,0);

            /* estado global */
            if(xSemaphoreTake(mutex_estado,portMAX_DELAY)){
                estado_sistema.temperatura_atual=t_filt;
                estado_sistema.temperatura_prevista=lin_prev;
                estado_sistema.temperatura_prevista_holt=holt_prev;
                xSemaphoreGive(mutex_estado);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_SEGUNDOS*1000));
    }
}

/*============================================================================
 *  TASK: ENTRADA USUÁRIO  (mesma lógica original)
 *===========================================================================*/
static void tarefa_in(void *p){
    (void)p; uint16_t adc_v; bool ba=false,bb=false; TickType_t ultJoy=0; ComandoUsuario_t cmd;
    while(1){
        adc_v=adc_read(); TickType_t now=xTaskGetTickCount(); EstadoSistema_t s; ler_estado(&s);
        if(s.tela_atual==0 && (now-ultJoy)>pdMS_TO_TICKS(DEBOUNCE_JOYSTICK_MS)){
            if(adc_v>3000){cmd.tipo=COMANDO_AJUSTAR_URGENCIA_SUBIR;cmd.valor=1;xQueueSend(q_cmd,&cmd,0);ultJoy=now;}
            else if(adc_v<1000){cmd.tipo=COMANDO_AJUSTAR_URGENCIA_DESCER;cmd.valor=-1;xQueueSend(q_cmd,&cmd,0);ultJoy=now;}
        }
        if(!gpio_get(PINO_BOTAO_A)&&!ba){ba=true;if(s.tela_atual==0){cmd.tipo=COMANDO_PROXIMA_TELA;xQueueSend(q_cmd,&cmd,0);beep(100,0,2000);}}
        else if(gpio_get(PINO_BOTAO_A))ba=false;
        if(!gpio_get(PINO_BOTAO_B)&&!bb){bb=true;if(s.tela_atual!=0){cmd.tipo=COMANDO_TELA_ANTERIOR;xQueueSend(q_cmd,&cmd,0);beep(100,0,2000);}}
        else if(gpio_get(PINO_BOTAO_B))bb=false;
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_BOTAO_MS));
    }
}

/*============================================================================
 *  TASK: DISPLAY
 *===========================================================================*/
static void tarefa_disp(void *p){
    (void)p; DadosTemperatura_t d; ResultadosPrevisao_t r; ComandoUsuario_t c;
    while(1){
        xTaskNotifyWait(0,ULONG_MAX,NULL,pdMS_TO_TICKS(TIMEOUT_ATUALIZACAO_DISPLAY_MS));
        if(xQueueReceive(q_temp,&d,0)==pdTRUE){
            if(xSemaphoreTake(mutex_estado,portMAX_DELAY)){estado_sistema.temperatura_atual=d.temperatura;xSemaphoreGive(mutex_estado);}
        }
        if(xQueueReceive(q_prev,&r,0)==pdTRUE){
            if(xSemaphoreTake(mutex_estado,portMAX_DELAY)){
                estado_sistema.temperatura_prevista=r.previsao_linear;
                estado_sistema.temperatura_prevista_holt=r.previsao_holt;
                xSemaphoreGive(mutex_estado);
            }
        }
        if(xQueueReceive(q_cmd,&c,0)==pdTRUE){
            if(xSemaphoreTake(mutex_estado,portMAX_DELAY)){
                switch(c.tipo){
                    case COMANDO_PROXIMA_TELA:estado_sistema.tela_atual=1;estado_sistema.configuracao_concluida=true;break;
                    case COMANDO_TELA_ANTERIOR:estado_sistema.tela_atual=0;estado_sistema.configuracao_concluida=false;break;
                    case COMANDO_AJUSTAR_URGENCIA_SUBIR:
                    case COMANDO_AJUSTAR_URGENCIA_DESCER:estado_sistema.temperatura_urgencia+=c.valor;break;
                }
                xSemaphoreGive(mutex_estado);
            }
        }
        EstadoSistema_t s; ler_estado(&s);
        const char *sit=situacao(s.temperatura_atual,s.temperatura_prevista,s.temperatura_urgencia);
        indicadores(sit,s.configuracao_concluida);
        if(xSemaphoreTake(mutex_display,portMAX_DELAY)){
            ssd1306_fill(&display,false);
            if(s.tela_atual==0) tela_cfg(s.temperatura_urgencia);
            else                tela_res(s.temperatura_atual,s.temperatura_prevista,
                                         s.temperatura_prevista_holt,s.temperatura_urgencia,sit);
            ssd1306_send_data(&display);
            xSemaphoreGive(mutex_display);
        }
    }
}

/*============================================================================
 *  TASK: PUBLICA MQTT
 *===========================================================================*/
static void pub_cb(void *a,err_t e){if(e)printf("pub err %d\n",e);}
static void tarefa_pub(void *p){
    (void)p; while(1){
        if(mqtt_state.conectado && mqtt_client_is_connected(mqtt_state.inst)){
            EstadoSistema_t s; ler_estado(&s); char buf[16];
            snprintf(buf,sizeof(buf),"%.2f",s.temperatura_atual);
            mqtt_publish(mqtt_state.inst,topic_full("/temperature"),buf,strlen(buf),
                         MQTT_PUBLISH_QOS,MQTT_PUBLISH_RETAIN,pub_cb,NULL);
            snprintf(buf,sizeof(buf),"%.2f",s.temperatura_prevista);
            mqtt_publish(mqtt_state.inst,topic_full("/temperature_predicted"),buf,strlen(buf),
                         MQTT_PUBLISH_QOS,MQTT_PUBLISH_RETAIN,pub_cb,NULL);
            snprintf(buf,sizeof(buf),"%.2f",s.temperatura_prevista_holt);
            mqtt_publish(mqtt_state.inst,topic_full("/temperature_predicted_holt"),buf,strlen(buf),
                         MQTT_PUBLISH_QOS,MQTT_PUBLISH_RETAIN,pub_cb,NULL);
            const char *sit=situacao(s.temperatura_atual,s.temperatura_prevista,s.temperatura_urgencia);
            mqtt_publish(mqtt_state.inst,topic_full("/status"),sit,strlen(sit),
                         MQTT_PUBLISH_QOS,MQTT_PUBLISH_RETAIN,pub_cb,NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(TEMP_PUBLISH_INTERVAL_S*1000));
    }
}

/*============================================================================
 *  MQTT CALLBACKS
 *===========================================================================*/
static void in_data(void *arg,const u8_t *data,u16_t len,u8_t flg){
    MQTT_STATE_t *st=(MQTT_STATE_t*)arg;
    strncpy(st->data,(const char*)data,MIN(len,sizeof(st->data)-1));st->data[len]='\0';
    if(strcmp(st->topic,topic_full("/led"))==0){
        bool on=(!strcasecmp(st->data,"on")||!strcmp(st->data,"1"));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,on);
    }else if(strcmp(st->topic,topic_full("/exit"))==0){mqtt_disconnect(st->inst);}
}
static void in_pub(void *arg,const char *topic,u32_t len){
    (void)len; MQTT_STATE_t *st=(MQTT_STATE_t*)arg; strncpy(st->topic,topic,sizeof(st->topic));
}
static void sub_cb(void *a,err_t e){if(e)printf("sub err %d\n",e);}
static void conn_cb(mqtt_client_t *c,void *arg,mqtt_connection_status_t stt){
    MQTT_STATE_t *st=(MQTT_STATE_t*)arg;
    if(stt==MQTT_CONNECT_ACCEPTED){
        printf("MQTT ok\n"); st->conectado=true;
        mqtt_sub_unsub(c,topic_full("/led"),MQTT_SUBSCRIBE_QOS,sub_cb,st,true);
        mqtt_sub_unsub(c,topic_full("/print"),MQTT_SUBSCRIBE_QOS,sub_cb,st,true);
        mqtt_sub_unsub(c,topic_full("/ping"),MQTT_SUBSCRIBE_QOS,sub_cb,st,true);
        mqtt_sub_unsub(c,topic_full("/exit"),MQTT_SUBSCRIBE_QOS,sub_cb,st,true);
        mqtt_publish(c,topic_full(MQTT_WILL_TOPIC),"1",1,MQTT_WILL_QOS,true,pub_cb,NULL);
    }else{printf("MQTT off %d\n",stt);st->conectado=false;}
}

/*============================================================================
 *  TASK: WI-FI + MQTT
 *===========================================================================*/
static void tarefa_wifi(void *p){
    (void)p;
    if(cyw43_arch_init()){printf("CYW43 err\n");vTaskDelete(NULL);}
    cyw43_arch_enable_sta_mode();
    printf("Wi-Fi %s...\n",WIFI_SSID);
    if(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID,WIFI_PASSWORD,CYW43_AUTH_WPA2_AES_PSK,30000)){
        printf("Wi-Fi falha\n");vTaskDelete(NULL);}
    printf("IP %s\n",ipaddr_ntoa(&(netif_list->ip_addr)));
    mqtt_state.inst=mqtt_client_new(); if(!mqtt_state.inst){vTaskDelete(NULL);}
    static char cid[12]; pico_get_unique_board_id_string(cid,sizeof(cid));
    for(int i=0;i<strlen(cid);i++)cid[i]=tolower(cid[i]);
    mqtt_state.info.client_id=cid; mqtt_state.info.keep_alive=MQTT_KEEP_ALIVE_S;
    mqtt_state.info.client_user=MQTT_USERNAME; mqtt_state.info.client_pass=MQTT_PASSWORD;
    mqtt_state.info.will_topic=topic_full(MQTT_WILL_TOPIC); mqtt_state.info.will_msg=MQTT_WILL_MSG;
    mqtt_state.info.will_qos=MQTT_WILL_QOS; mqtt_state.info.will_retain=true;
    if(dns_gethostbyname(MQTT_SERVER,&mqtt_state.server_addr,NULL,NULL)!=ERR_OK){
        while(dns_gethostbyname(MQTT_SERVER,&mqtt_state.server_addr,NULL,NULL)==ERR_INPROGRESS) vTaskDelay(pdMS_TO_TICKS(500));}
    printf("Broker %s\n",ipaddr_ntoa(&mqtt_state.server_addr));
    mqtt_set_inpub_callback(mqtt_state.inst,in_pub,in_data,&mqtt_state);
    if(mqtt_client_connect(mqtt_state.inst,&mqtt_state.server_addr,MQTT_PORT,conn_cb,&mqtt_state,&mqtt_state.info)!=ERR_OK){
        printf("MQTT connect err\n");vTaskDelete(NULL);}
    while(1){cyw43_arch_poll();vTaskDelay(pdMS_TO_TICKS(10));}
}

/*============================================================================
 *  MAIN
 *===========================================================================*/
int main(void){
    stdio_init_all();
    /* I2C Display */
    i2c_init(i2c1,400*1000);
    gpio_set_function(PINO_SDA_I2C,GPIO_FUNC_I2C);
    gpio_set_function(PINO_SCL_I2C,GPIO_FUNC_I2C);
    gpio_pull_up(PINO_SDA_I2C); gpio_pull_up(PINO_SCL_I2C);
    ssd1306_init(&display,128,64,false,0x3C,i2c1); ssd1306_config(&display);
    /* ADC / botões / LEDs / buzzer */
    adc_init(); adc_gpio_init(PINO_ADC); adc_select_input(0);
    gpio_init(PINO_BOTAO_A); gpio_set_dir(PINO_BOTAO_A,GPIO_IN); gpio_pull_up(PINO_BOTAO_A);
    gpio_init(PINO_BOTAO_B); gpio_set_dir(PINO_BOTAO_B,GPIO_IN); gpio_pull_up(PINO_BOTAO_B);
    gpio_init(PINO_LED_VERDE); gpio_set_dir(PINO_LED_VERDE,GPIO_OUT);
    gpio_init(PINO_LED_VERMELHO); gpio_set_dir(PINO_LED_VERMELHO,GPIO_OUT);
    gpio_set_function(PINO_BUZZER,GPIO_FUNC_PWM);
    slice_buzzer=pwm_gpio_to_slice_num(PINO_BUZZER); channel_buzzer=pwm_gpio_to_channel(PINO_BUZZER);
    pwm_set_clkdiv(slice_buzzer,125.0f); pwm_set_wrap(slice_buzzer,(1000000/2000)-1);
    pwm_set_chan_level(slice_buzzer,channel_buzzer,(1000000/2000)/2); pwm_set_enabled(slice_buzzer,false);
    ds18b20_init(PINO_DS18B20); inicializar_matriz_led();

    /* RTOS infra */
    mutex_estado=xSemaphoreCreateMutex(); mutex_display=xSemaphoreCreateMutex();
    q_temp=xQueueCreate(10,sizeof(DadosTemperatura_t));
    q_prev=xQueueCreate(10,sizeof(ResultadosPrevisao_t));
    q_cmd =xQueueCreate(10,sizeof(ComandoUsuario_t));

    xTaskCreate(tarefa_temp,"Temp",   1024,NULL,2,NULL);
    xTaskCreate(tarefa_in,  "Entrada",512,NULL,1,NULL);
    xTaskCreate(tarefa_disp,"Display",1024,NULL,1,NULL);
    xTaskCreate(tarefa_wifi,"WiFi",   2048,NULL,3,NULL);
    xTaskCreate(tarefa_pub, "MQTTPub",768,NULL,1,NULL);

    vTaskStartScheduler();
    while(1) tight_loop_contents();
    return 0;
}
