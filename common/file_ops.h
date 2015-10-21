#define RANDOM_DATA_POOL "random_data"

extern void file_read_random_data(const char *prefix);
extern uint16_t file_get_random_data(int device_id, int channel_id);
extern int file_get_highest_channel(void);
extern void file_print_random_data();
extern void file_update_regs(uint16_t *regs, int device_id);

extern void file_device_enumerate(void (*add_device_func)(int device_id));
extern void file_channel_enumerate(
	void (*add_channel_func)(size_t num_words, uint16_t *data, void *arg),
	void *arg);

extern int file_num_devices(void);
extern void file_free_random_data();
