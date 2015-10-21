#include <stdio.h>
#include <stdlib.h>
#include <libbacnet/address.h>
#include <libbacnet/device.h>
#include <libbacnet/handlers.h>
#include <libbacnet/datalink.h>
#include <libbacnet/bvlc.h>
#include <libbacnet/client.h>
#include <libbacnet/txbuf.h>
#include <libbacnet/tsm.h>
#include <libbacnet/ai.h>
#include "bacnet_namespace.h"
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <modbus-tcp.h>
#include <errno.h>

#define BACNET_INSTANCE_NO	    48 
// Use 12 initial tests, 48 for VU testing (random_data folder)

#define BACNET_PORT		    0xBAC1
#define BACNET_INTERFACE	    "lo"
#define BACNET_DATALINK_TYPE	    "bvlc"
#define BACNET_SELECT_TIMEOUT_MS    1	    /* ms */

#define RUN_AS_BBMD_CLIENT	    1

#if RUN_AS_BBMD_CLIENT
#define BACNET_BBMD_PORT	    0xBAC0

//#define BACNET_BBMD_ADDRESS	    "127.0.0.1" 
#define BACNET_BBMD_ADDRESS	    "140.159.160.7" 
// Initially 127.0.0.1, change to 140.159.160.7 for VU

#define BACNET_BBMD_TTL		    90
#endif

/*static uint16_t test_data[] = 
{
    0xA4EC, 0x6E39, 0x8740, 0x1065, 0x9134, 0xFC8C
};*/
// Need this to change to read the incoming data itself, not read pre made data.


static pthread_mutex_t timer_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t list_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t list_data_ready = PTHREAD_COND_INITIALIZER; // added for when link list is installed
static pthread_cond_t list_data_flush = PTHREAD_COND_INITIALIZER;  // added for when link list is installed

uint16_t thread_display[3] ={};

typedef struct s_word_object word_object;
struct s_word_object
{
	uint16_t value;
	word_object *next;
};

static word_object *list_head[2];

/* Add object to list */
static void add_to_list(word_object **list_head, uint16_t value) 
{
	word_object *last_object, *tmp_object;
	/* Do all memory allocation outside of locking - strdup() and malloc() can block */
	tmp_object = malloc(sizeof(word_object));
	/* Set up tmp_object outside of locking */
	tmp_object -> value = value;
	tmp_object -> next = NULL;
	pthread_mutex_lock(&list_lock);
	
	if (*list_head == NULL) 
	{
		/* The list is empty, just place our tmp_object at the head */
		*list_head = tmp_object;
	} 
	
	else 
	{
		/* Iterate through the linked list to find the last object */
		last_object = *list_head;
	
		while (last_object->next) 
		{
			last_object = last_object->next;
		}

		/* Last object is now found, link in our tmp_object at the tail */
		last_object->next = tmp_object;
	}
	pthread_mutex_unlock(&list_lock);
	pthread_cond_signal(&list_data_ready);
}

static word_object *list_get_first(word_object **list_head) 
{
	word_object *first_object;
	first_object = *list_head;
	*list_head = (*list_head) -> next;
	return first_object;
}

static int Update_Analog_Input_Read_Property(BACNET_READ_PROPERTY_DATA *rpdata) 
{
    static int index;
    word_object *object;
    int instance_no = Analog_Input_Instance_To_Index(rpdata -> object_instance);

	if(rpdata->object_property != bacnet_PROP_PRESENT_VALUE) 
	{
		pthread_mutex_lock(&list_lock);
    		goto not_pv;
	}

	if(list_head[instance_no] == NULL)
	{
		pthread_mutex_unlock(&list_lock);
		goto not_pv;
	}
	
	object = list_get_first(&list_head[instance_no]);
	printf("AI_Present_Value request for instance %i\n", instance_no);
    	thread_display[instance_no] = object -> value;
	//bacnet_Analog_Input_Present_Value_Set(0, 20);
	free(object);
	bacnet_Analog_Input_Present_Value_Set(instance_no, thread_display[instance_no]);
	not_pv:
	return bacnet_Analog_Input_Read_Property(rpdata);
}

static bacnet_object_functions_t server_objects[] = {
    {bacnet_OBJECT_DEVICE,
	    NULL,
	    bacnet_Device_Count,
	    bacnet_Device_Index_To_Instance,
	    bacnet_Device_Valid_Object_Instance_Number,
	    bacnet_Device_Object_Name,
	    bacnet_Device_Read_Property_Local,
	    bacnet_Device_Write_Property_Local,
	    bacnet_Device_Property_Lists,
	    bacnet_DeviceGetRRInfo,
	    NULL, /* Iterator */
	    NULL, /* Value_Lists */
	    NULL, /* COV */
	    NULL, /* COV Clear */
	    NULL  /* Intrinsic Reporting */
    },
    {bacnet_OBJECT_ANALOG_INPUT,
            bacnet_Analog_Input_Init,
            bacnet_Analog_Input_Count,
            bacnet_Analog_Input_Index_To_Instance,
            bacnet_Analog_Input_Valid_Instance,
            bacnet_Analog_Input_Object_Name,
            Update_Analog_Input_Read_Property,
            bacnet_Analog_Input_Write_Property,
            bacnet_Analog_Input_Property_Lists,
            NULL /* ReadRangeInfo */ ,
            NULL /* Iterator */ ,
            bacnet_Analog_Input_Encode_Value_List,
            bacnet_Analog_Input_Change_Of_Value,
            bacnet_Analog_Input_Change_Of_Value_Clear,
            bacnet_Analog_Input_Intrinsic_Reporting},
    {MAX_BACNET_OBJECT_TYPE}
};

static void register_with_bbmd(void) 
{
    #if RUN_AS_BBMD_CLIENT
    /* Thread safety: Shares data with datalink_send_pdu */
    bacnet_bvlc_register_with_bbmd(
    bacnet_bip_getaddrbyname(BACNET_BBMD_ADDRESS), 
    htons(BACNET_BBMD_PORT),
    BACNET_BBMD_TTL);
    #endif
}

static void *minute_tick(void *arg) 
{
    while (1) 
    {
	pthread_mutex_lock(&timer_lock);

	/* Expire addresses once the TTL has expired */
	bacnet_address_cache_timer(60);

	/* Re-register with BBMD once BBMD TTL has expired */
	register_with_bbmd();

	/* Update addresses for notification class recipient list 
	 * Requred for INTRINSIC_REPORTING
	 * bacnet_Notification_Class_find_recipient(); */
	
	/* Sleep for 1 minute */
	pthread_mutex_unlock(&timer_lock);
	sleep(60);
    }

    return arg;
}

static void *second_tick(void *arg) 
{
    while (1) 
    {
	pthread_mutex_lock(&timer_lock);

	/* Invalidates stale BBMD foreign device table entries */
	bacnet_bvlc_maintenance_timer(1);

	/* Transaction state machine: Responsible for retransmissions and ack
	 * checking for confirmed services */
	bacnet_tsm_timer_milliseconds(1000);

	/* Re-enables communications after DCC_Time_Duration_Seconds
	 * Required for SERVICE_CONFIRMED_DEVICE_COMMUNICATION_CONTROL
	 * bacnet_dcc_timer_seconds(1); */

	/* State machine for load control object
	 * Required for OBJECT_LOAD_CONTROL
	 * bacnet_Load_Control_State_Machine_Handler(); */

	/* Expires any COV subscribers that have finite lifetimes
	 * Required for SERVICE_CONFIRMED_SUBSCRIBE_COV
	 * bacnet_handler_cov_timer_seconds(1); */

	/* Monitor Trend Log uLogIntervals and fetch properties
	 * Required for OBJECT_TRENDLOG
	 * bacnet_trend_log_timer(1); */
	
	/* Run [Object_Type]_Intrinsic_Reporting() for all objects in device
	 * Required for INTRINSIC_REPORTING
	 * bacnet_Device_local_reporting(); */
	
	/* Sleep for 1 second */
	pthread_mutex_unlock(&timer_lock);
	sleep(1);
    }

    return arg;
}

static void ms_tick(void) 
{
    /* Updates change of value COV subscribers.
     * Required for SERVICE_CONFIRMED_SUBSCRIBE_COV
     * bacnet_handler_cov_task(); */
}

#define BN_UNC(service, handler) \
    bacnet_apdu_set_unconfirmed_handler(		\
		    SERVICE_UNCONFIRMED_##service,	\
		    bacnet_handler_##handler)
#define BN_CON(service, handler) \
    bacnet_apdu_set_confirmed_handler(			\
		    SERVICE_CONFIRMED_##service,	\
		    bacnet_handler_##handler)

static void *modbus_side (void *arg) //_______________________________________________________________________static void *modbus_side (void *arg)
{
	modbus_t *ctx;
	uint16_t tab_reg[64];
	int rc;
	int i;
modbus_side_restart:
	
	//ctx = modbus_new_tcp("127.0.0.1", 502);
	ctx = modbus_new_tcp("140.159.153.159", 502);
	// Use 140.159.153.159 for uni, 127.0.0.1 for home. Server port remains as 502 for both cases
	
	// Initialize connection to modbus server
	if(modbus_connect(ctx) == -1)  // Connection to server failed
	{
		fprintf(stderr, "Connection to server failed: %s\n", modbus_strerror(errno));
		modbus_free(ctx); 
		modbus_close(ctx);
		usleep(100000);  // Sleep for suggested delay time of 100ms
		// Closing connection to retry again
		goto modbus_side_restart;
	}

	else  // Connection to server successful
	{
		fprintf(stderr, "Connection to server successful\n");
	
	}

	// Read registers
	while(1)
	{
		rc = modbus_read_registers(ctx, 48, 2, tab_reg);  // Device 48 allocated for Nick Hodson

		if(rc == -1)  // Read of registers failed
		{
			fprintf(stderr, "Reading of registers has failed: %s\n", modbus_strerror(errno));

		// CLose modbus connection and start again (retry)
		modbus_free(ctx); 
		modbus_close(ctx);
		goto modbus_side_restart;
		}

		// not putting an else statement for succcessful read register as it will clog up the terminal window

		for(i = 0; i < rc; i++)  // Register display
		{
			add_to_list(&list_head[i], tab_reg[i]);  // replacement for printf statement
			printf("register[%d] = %d (0x%X)\n", i, tab_reg[i], tab_reg[i]);
		}	
		usleep(100000);  // Sleep for suggested delay time of 100ms
	}
}	

int main(int argc, char **argv) //_____________________________________________________________________int main(int argc, char **argv)
{
    uint8_t rx_buf[bacnet_MAX_MPDU];
    uint16_t pdu_len;
    BACNET_ADDRESS src;
    pthread_t minute_tick_id, second_tick_id, modbus_side_id;  // Used for creation of threads

    bacnet_Device_Set_Object_Instance_Number(BACNET_INSTANCE_NO);
    bacnet_address_init();

    /* Setup device objects */
    bacnet_Device_Init(server_objects);
    BN_UNC(WHO_IS, who_is);
    BN_CON(READ_PROPERTY, read_property);

    bacnet_BIP_Debug = true;
    bacnet_bip_set_port(htons(BACNET_PORT));
    bacnet_datalink_set(BACNET_DATALINK_TYPE);
    bacnet_datalink_init(BACNET_INTERFACE);
    atexit(bacnet_datalink_cleanup);
    memset(&src, 0, sizeof(src));

    register_with_bbmd();

    bacnet_Send_I_Am(bacnet_Handler_Transmit_Buffer);

    pthread_create(&minute_tick_id, 0, minute_tick, NULL);  // Create thread for minute_tick function
    pthread_create(&second_tick_id, 0, second_tick, NULL);  // Create thread for second_tick function
    pthread_create(&modbus_side_id, 0, modbus_side, NULL);  // Create thread for modbus_side function 

    while (1) 
    {
	pdu_len = bacnet_datalink_receive(
		    &src, rx_buf, bacnet_MAX_MPDU, BACNET_SELECT_TIMEOUT_MS);

	if (pdu_len) 
	{
	    /* May call any registered handler.
	     * Thread safety: May block, however we still need to guarantee
	     * atomicity with the timers, so hold the lock anyway */
	    pthread_mutex_lock(&timer_lock);
	    bacnet_npdu_handler(&src, rx_buf, pdu_len);
	    pthread_mutex_unlock(&timer_lock);
	}

	ms_tick();
    }

    return 0;
}
