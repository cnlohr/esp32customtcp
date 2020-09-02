#include <tcpip_adapter.h>
#include <cnip_hal.h>
#include <cnip_core.h>
#include <cnip_dhcp.h>
//#include <esp_eth.h>
//#include <esp_wifi_internal.h>
#include <esp_private/wifi.h>
#include <string.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi_netif.h>

volatile uint32_t  packet_stopwatch;

static const char * tag __attribute__((used)) = "tcpip_adapter";
#define NUM_DEVS 3

cnip_hal haldevs[NUM_DEVS]; //index with esp_interface_t


const esp_netif_netstack_config_t *_g_esp_netif_netstack_default_eth      = (void*)&haldevs[2];
const esp_netif_netstack_config_t *_g_esp_netif_netstack_default_wifi_sta = (void*)&haldevs[0];
const esp_netif_netstack_config_t *_g_esp_netif_netstack_default_wifi_ap  = (void*)&haldevs[1];
uint8_t  interface_up[NUM_DEVS];
/*
	typedef enum {
		ESP_IF_WIFI_STA = 0,
		ESP_IF_WIFI_AP,
		ESP_IF_ETH,
		ESP_IF_MAX
	} esp_interface_t;
*/

//Why doesn't weak aliasing seem to work here?
void cnip_user_init( cnip_hal * hal );
void cnip_user_timed_tick( cnip_hal * hal, int is_slow );
void cnip_handle_udp( cnip_ctx * ctx, uint16_t len );
uint8_t cnip_tcp_recv_syn_cb( struct cnip_ctx_t * ctx, uint16_t portno );
uint8_t cnip_tcp_recv_data_cb( cnip_tcp * tcp, int sockno, uint16_t totallen );
void cnip_tcp_connection_closing_cb( cnip_tcp * tcp, int sockno );


void cnip_task( void * pvParameters )
{
	int i;
	TickType_t last_tick = xTaskGetTickCount();


    for( ;; )
    {		
		TickType_t current_tick = xTaskGetTickCount();
		if( current_tick - last_tick > 20 )
		{
			for( i = 0; i < 3; i++ )
			{
				cnip_hal * hal = &haldevs[i];
				if( interface_up[i] )
				{
					//Perform slow ops
					xSemaphoreTake( hal->host_lock, portMAX_DELAY );
					cnip_tick_dhcp( hal->ip_ctx );
					cnip_user_timed_tick( &haldevs[i], 1 );
					cnip_tcp_tick( hal->ip_ctx );
#if 0
					printf( "T:" );
					int j;
					for( j = 0; j < 10; j++ )
						printf( "%d(%d) ", hal->ip_ctx->TCPs[j].state, hal->ip_ctx->TCPs[j].time_since_sent );
					printf( "\n" );
#endif
					xSemaphoreGive( hal->host_lock );
				}
			}
			last_tick = current_tick;
		}

		for( i = 0; i < 3; i++ )
		{
			if( interface_up[i] )
			{
				cnip_hal * hal = &haldevs[i];
				xSemaphoreTake( hal->host_lock, portMAX_DELAY );
				cnip_user_timed_tick( hal, 0 );
				xSemaphoreGive( hal->host_lock );
			}
		}
		//Perform high-frequency ops.
		vTaskDelay(1);
    }
}



void tcpip_adapter_init(void)
{
	xTaskCreate( cnip_task,
         "cnip",
         2048, //Should not need to store an MTU.
         0,
         1,
         0
       );

#if 0
	xTaskCreatePinnedToCore(
                    cnip_task,   /* Function to implement the task */
                    "cnip", /* Name of the task */
                    2048,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    0 );  /* Core where the task should run */
#endif

}


//XXX TODO: Handle esp_wifi_internal_set_sta_ip

static esp_err_t CNIP_IRAM send_cnip( cnip_hal * hal, void * buffer, uint16_t len )
{
	xSemaphoreTake( hal->host_lock, portMAX_DELAY );
	hal->incoming_cur = hal->incoming_base = buffer;
	hal->incoming_end = buffer + len;
	packet_stopwatch = GetCycleCount();
	cnip_hal_receivecallback( hal );
	xSemaphoreGive( hal->host_lock );
	return ESP_OK;
}

esp_err_t CNIP_IRAM tcpip_adapter_eth_input(void *buffer, uint16_t len, void *eb)
{
	int ret = 0;
	if( interface_up[ESP_IF_ETH] )
		ret = send_cnip( &haldevs[ESP_IF_ETH], buffer, len );
	esp_wifi_internal_free_rx_buffer(eb);
	return ret;
}

esp_err_t CNIP_IRAM tcpip_adapter_sta_input(void *buffer, uint16_t len, void *eb)
{
	int ret = 0;

	if( interface_up[ESP_IF_WIFI_STA] )
		ret = send_cnip( &haldevs[ESP_IF_WIFI_STA], buffer, len );
	esp_wifi_internal_free_rx_buffer(eb);
	return ret;
}

esp_err_t CNIP_IRAM tcpip_adapter_ap_input(void *buffer, uint16_t len, void *eb)
{
	int ret = 0;

	if( interface_up[ESP_IF_WIFI_AP] )
		ret = send_cnip( &haldevs[ESP_IF_WIFI_AP], buffer, len );
	esp_wifi_internal_free_rx_buffer(eb);
	return ret;
}


int esp_eth_tx( cnip_mem_address start, uint16_t len );

/* For sending things the other way... */
int8_t CNIP_IRAM cnip_hal_xmitpacket( cnip_hal * hal, cnip_mem_address start, uint16_t len )
{
	int8_t ret;

	//This is where we pass our packet back to wifi.

	if( hal->host == (void*)ESP_IF_ETH )
		return -1;//ret = esp_eth_tx( start, len ); FIXME At some point.
	else
	{
		ret = esp_wifi_internal_tx( (wifi_interface_t)hal->host, start, len );
		if( hal->host == 0 )
		{
			//XXX TODO: Figure out when we're in station mode, we can't send DHCP packets!? 
			printf( "Sending: HOST: %p %p %d = %d\n", hal->host, start, len, ret );
			int i;
			for( i = 0; i < len; i++ )
			{
				printf( "%02x%c", ((char*)start)[i], ((i&0xf)==0xf)?'\n':' ' );
			}
			printf( "\n" );
		}
	}
	return ret;
}


/* Calls from IP stack */

void cnip_got_dhcp_lease_cb( cnip_ctx * ctx )
{
	printf( "Got Lease!!\n" );
	uint8_t * ip = (uint8_t*)&ctx->ip_addr;
	printf( "IP: %d.%d.%d.%d  %08x\n", ip[0], ip[1], ip[2], ip[3], ctx->ip_addr );
}

static void tcpip_adapter_action_sta_connected(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    uint8_t mac[6];
	tcpip_adapter_ip_info_t ipinfo;
//    wifi_netif_driver_t driver = esp_netif_get_io_driver(esp_netif);
//	esp_wifi_get_if_mac(driver, mac );
	esp_wifi_get_mac( WIFI_IF_AP, mac );
	tcpip_adapter_sta_start(mac, &ipinfo);

	interface_up[ESP_IF_WIFI_STA] = 1;
}

static void tcpip_adapteraction_sta_disconnected(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
	//Tell it to reconnect? Is this needed?
    esp_wifi_connect();
	interface_up[ESP_IF_WIFI_STA] = 0;
}

static void wifi_create_and_start_sta(void *esp_netif, esp_event_base_t base, int32_t event_id, void *data)
{
	esp_wifi_internal_reg_rxcb( WIFI_IF_STA, tcpip_adapter_sta_input );
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, tcpip_adapter_action_sta_connected, NULL);
	esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, tcpip_adapteraction_sta_disconnected, NULL);
	esp_wifi_connect();

	//XXX This is a half-hearted attempt overall, we actually need to register more events and handle their failure.
}

static void wifi_create_and_start_ap(void *esp_netif, esp_event_base_t base, int32_t event_id, void *data)
{
    uint8_t mac[6];
	tcpip_adapter_ip_info_t ipinfo;
 // wifi_netif_driver_t driver = esp_netif_get_io_driver(esp_netif);
 //	esp_wifi_get_if_mac(driver, mac );
	esp_wifi_get_mac( WIFI_IF_AP, mac );
	tcpip_adapter_ap_start(mac, &ipinfo);
	esp_wifi_internal_reg_rxcb( WIFI_IF_AP, tcpip_adapter_ap_input );
}


esp_err_t tcpip_adapter_compat_start_eth(void* eth_driver)
{
	printf( "tcpip_adapter_compat_start_eth!!\n" );
    return ESP_OK;
}


esp_err_t tcpip_adapter_set_default_wifi_handlers(void)
{
    // create instances and register default handlers only on start event
    esp_err_t err = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, wifi_create_and_start_sta, NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, wifi_create_and_start_ap, NULL);
    if (err != ESP_OK) {
        return err;
    }

	printf( "TODO: Start Ethernet, too!\n" );

    return ESP_OK;
}

esp_err_t tcpip_adapter_clear_default_wifi_handlers(void)
{
    // No action if tcpip-adapter compatibility enabled, but interfaces created/configured with esp-netif
    return ESP_OK;
}



//We need to do some device resetting here.
esp_err_t tcpip_adapter_eth_start(uint8_t *mac, tcpip_adapter_ip_info_t *ip_info)
{
	printf( "Starting ETH adapter.\n" );
	cnip_hal * hal = &haldevs[ESP_IF_ETH];
	cnip_hal_init( hal, mac );
	hal->host = (void*)ESP_IF_ETH;
	hal->host_lock = xSemaphoreCreateBinary( );
	xSemaphoreGive( hal->host_lock );
//	tcpip_adapter_set_ip_info( ESP_IF_ETH, ip_info );
	cnip_init_ip( hal->ip_ctx );
	cnip_user_init( hal );
	return ESP_OK;
}

esp_err_t tcpip_adapter_sta_start(uint8_t *mac, tcpip_adapter_ip_info_t *ip_info)
{
	printf( "Starting STA adapter.\n" );
	cnip_hal * hal = &haldevs[ESP_IF_WIFI_STA];
	cnip_hal_init( hal, mac );
	hal->host = (void*)ESP_IF_WIFI_STA;
	hal->host_lock = xSemaphoreCreateBinary( );
	xSemaphoreGive( hal->host_lock );
//	tcpip_adapter_set_ip_info( ESP_IF_WIFI_STA, ip_info );
	cnip_init_ip( hal->ip_ctx );
	cnip_dhcpc_create( hal->ip_ctx );
	cnip_user_init( hal );
	return ESP_OK;
}

esp_err_t tcpip_adapter_ap_start(uint8_t *mac, tcpip_adapter_ip_info_t *ip_info)
{
	printf( "Starting AP adapter\n" );
	cnip_hal * hal = &haldevs[ESP_IF_WIFI_AP];

	cnip_hal_init( hal, mac );
	hal->host = (void*)ESP_IF_WIFI_AP;
	hal->host_lock = xSemaphoreCreateBinary( );
	xSemaphoreGive( hal->host_lock );
//	tcpip_adapter_set_ip_info( ESP_IF_WIFI_AP, ip_info );
	cnip_ctx * ctx = hal->ip_ctx;
	cnip_init_ip( ctx );
	ctx->ip_addr = CNIPIP( 192, 168, 4, 1 );
	ctx->ip_mask = CNIPIP( 255, 255, 255, 0 );
	ctx->ip_gateway = CNIPIP( 192, 168, 4, 1 );
	cnip_dhcps_create( ctx );

	//This interface doesn't get the adapter_up flag.
	interface_up[ESP_IF_WIFI_AP] = 1; 

	cnip_user_init( hal );
	return ESP_OK;
}


esp_err_t tcpip_adapter_stop(tcpip_adapter_if_t tcpip_if)
{
	//TODO: add a teardown.
	return ESP_OK;
}

esp_err_t tcpip_adapter_up(tcpip_adapter_if_t tcpip_if)   { printf( "IF %d UP\n", tcpip_if ); interface_up[tcpip_if] = 1; return ESP_OK; }
esp_err_t tcpip_adapter_down(tcpip_adapter_if_t tcpip_if) { printf( "IF %d DOWN\n", tcpip_if ); interface_up[tcpip_if] = 0; return ESP_OK; }

esp_err_t tcpip_adapter_get_ip_info(tcpip_adapter_if_t tcpip_if, tcpip_adapter_ip_info_t *ip_info)
{
	if( haldevs[tcpip_if].ip_ctx )
	{
		memcpy( ip_info, &haldevs[tcpip_if].ip_ctx->ip_addr, sizeof( *ip_info ) ); 
		return ESP_OK;
	}
	else
	{
		//For some reason this is normally called before initialization, so we have to fail out of here.
		return ESP_ERR_NOT_FOUND;
	}
}

esp_err_t tcpip_adapter_set_ip_info(tcpip_adapter_if_t tcpip_if, const tcpip_adapter_ip_info_t *ip_info)
{
	memcpy( &haldevs[tcpip_if].ip_ctx->ip_addr, ip_info, sizeof( *ip_info ) ); 
	return ESP_OK;
}

esp_err_t tcpip_adapter_set_dns_info(tcpip_adapter_if_t tcpip_if, tcpip_adapter_dns_type_t type, tcpip_adapter_dns_info_t *dns) { printf( "DNS NNS\n" ); return ESP_ERR_NOT_SUPPORTED; }
esp_err_t tcpip_adapter_get_dns_info(tcpip_adapter_if_t tcpip_if, tcpip_adapter_dns_type_t type, tcpip_adapter_dns_info_t *dns) { printf( "DNS NNS\n" ); return ESP_ERR_NOT_SUPPORTED; }

esp_err_t tcpip_adapter_get_old_ip_info(tcpip_adapter_if_t tcpip_if, tcpip_adapter_ip_info_t *ip_info)
{
	printf( "tcpip_adapter_get_old_ip_info STUB\n" );
	//XXX TODO
	return ESP_OK;
}

esp_err_t tcpip_adapter_set_old_ip_info(tcpip_adapter_if_t tcpip_if, const tcpip_adapter_ip_info_t *ip_info)
{
	printf( "tcpip_adapter_set_old_ip_info STUB\n" );
	//XXX TODO
	return ESP_OK;
}

esp_err_t tcpip_adapter_create_ip6_linklocal(tcpip_adapter_if_t tcpip_if)
{
	printf( "IPv6 Not supported.\n" );
	return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t tcpip_adapter_get_ip6_linklocal(tcpip_adapter_if_t tcpip_if, ip6_addr_t *if_ip6)
{
	printf( "IPv6 Not supported.\n" );
	return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t tcpip_adapter_dhcps_get_status(tcpip_adapter_if_t tcpip_if, tcpip_adapter_dhcp_status_t *status)
{
	printf( "tcpip_adapter_dhcps_get_status STUB\n" );
	//XXX TODO
	return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t tcpip_adapter_dhcps_option(tcpip_adapter_dhcp_option_mode_t opt_op, tcpip_adapter_dhcp_option_id_t opt_id, void *opt_val, uint32_t opt_len)
{
	printf( "tcpip_adapter_dhcps_option STUB\n" );
	//XXX TODO
	return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t tcpip_adapter_dhcps_start(tcpip_adapter_if_t tcpip_if)
{
	printf( "tcpip_adapter_dhcps_start STUB\n" );
	//XXX TODO
	return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t tcpip_adapter_dhcps_stop(tcpip_adapter_if_t tcpip_if)
{
	printf( "tcpip_adapter_dhcps_stop STUB\n" );
	//XXX TODO
	return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t tcpip_adapter_dhcpc_get_status(tcpip_adapter_if_t tcpip_if, tcpip_adapter_dhcp_status_t *status)
{
	printf( "tcpip_adapter_dhcpc_get_status STUB\n" );
	//XXX TODO
	return ESP_OK;
}

esp_err_t tcpip_adapter_dhcpc_option(tcpip_adapter_dhcp_option_mode_t opt_op, tcpip_adapter_dhcp_option_id_t opt_id, void *opt_val, uint32_t opt_len)
{
	printf( "tcpip_adapter_dhcpc_option STUB\n" );
	//XXX TODO
	return ESP_OK;
}

esp_err_t tcpip_adapter_dhcpc_start(tcpip_adapter_if_t tcpip_if)
{
	printf( "tcpip_adapter_dhcpc_start STUB\n" );
	//XXX TODO
	return ESP_OK;
}

esp_err_t tcpip_adapter_dhcpc_stop(tcpip_adapter_if_t tcpip_if)
{
	printf( "tcpip_adapter_dhcpc_stop STUB\n" );
	//XXX TODO
	return ESP_OK;
}

esp_interface_t tcpip_adapter_get_esp_if(void *dev)
{
	printf( "tcpip_adapter_get_esp_if STUB\n" );

	//XXX TODO
	//XXX TODO
	return 0;
}

esp_err_t tcpip_adapter_get_sta_list(const wifi_sta_list_t *wifi_sta_list, tcpip_adapter_sta_list_t *tcpip_sta_list)
{
	printf( "tcpip_adapter_get_sta_list STUB\n" );

	return ESP_ERR_NOT_SUPPORTED;
}


