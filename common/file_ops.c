#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <dirent.h>
#include <stdint.h>
#include <string.h>

#include "list.h"
#include "file_ops.h"

static LIST_HEAD(devices);

static int num_devices;

typedef struct random_channel_data_s random_channel_obj;
struct random_channel_data_s {
    size_t num_words;
    size_t index;
    uint16_t *data;

    list_entry channels;
};

typedef struct random_device_data_s random_device_obj;
struct random_device_data_s {
    int device_id;
    int num_regs;

    struct list_head channels;

    list_entry devices;
};

static random_device_obj *enum_device;

static int file_filter(const struct dirent *entry) {
    if (!strcmp(entry->d_name, "..")) return 0;
    if (entry->d_name[0] == '.') return 0;
    if (entry->d_type == DT_DIR) return 1;
    if (entry->d_type == DT_REG) return 1;
    return 0;
}

static int file_sorter(const struct dirent **a, const struct dirent **b) {
    if (((*a)->d_type == DT_DIR) && ((*b)->d_type == DT_REG)) return -1;
    if (((*a)->d_type == DT_REG) && ((*b)->d_type == DT_DIR)) return 1;

    return strverscmp ((*a)->d_name, (*b)->d_name);
}

static void add_device_data(const char *location, random_device_obj *device) {
    int n_members, i, j;
    struct dirent **dir_contents;
    random_channel_obj *channel;
    char *data_filename;
    FILE *fp;
    size_t file_size, bytes;

    INIT_LIST_HEAD(&device->channels);
    device->num_regs = 0;

    n_members = scandir(location, &dir_contents, file_filter, file_sorter);

    for (i = 0; i < n_members; i++) {
	device->num_regs++;

	channel = malloc(sizeof(random_channel_obj));
	list_add_tail(&channel->channels, &device->channels);

	if (asprintf(&data_filename, "%s/%s",
			location, dir_contents[i]->d_name) < 0) {
	    fprintf(stderr, "Error allocating filename\n");
	    exit(1);
	}

	fp = fopen(data_filename, "r");
	fseek(fp, 0, SEEK_END);
	file_size = ftell(fp);
	rewind(fp);

	/* File size should be aligned to uint16_t */
	if (file_size % 2) {
	    fprintf(stderr, "Illegal file size for %s\n", data_filename);
	    exit(1);
	}

	channel->data = malloc(file_size);
	bytes = fread(channel->data, 1, file_size, fp);

	if (bytes != file_size) {
	    fprintf(stderr, "Incomplete read for %s: %li\n", 
			    data_filename, bytes);
	    exit(1);
	}

	channel->num_words = file_size / 2;
	channel->index = 0;
	num_devices++;

	/* Check for duplicates, so we can discard them when the server's link
	 * list is empty */
	for (j = 1; j < channel->num_words; j++) {
	    if (channel->data[j] == channel->data[j - 1]) {
		fprintf(stderr, "Duplicate data found for %s\n", data_filename);
		exit(1);
	    }
	}

	fclose(fp);
	free(data_filename);
	free(dir_contents[i]);
    }
    free(dir_contents);
}

void file_read_random_data(const char *prefix) {
    /* Enumerate devices in random data directory */
    int n_members, i;
    struct dirent **dir_contents;
    random_device_obj *device;
    char *device_dir_name;

    n_members = scandir(prefix, &dir_contents, file_filter, file_sorter);

    if (n_members < 0) {
	fprintf(stderr, "Unable to open random data directory: %s\n", prefix);
	exit(1);
    }

    for (i = 0; i < n_members; i++) {
	/* enter each subdir and read out random data */
	device = malloc(sizeof(random_device_obj));
	
	device->device_id = atoi(dir_contents[i]->d_name);

	if (!device->device_id) {
	    fprintf(stderr, "Illegal device number %s\n",
			    dir_contents[i]->d_name);
	    exit(1);
	}

	if (asprintf(&device_dir_name, "%s/%s", 
				prefix, dir_contents[i]->d_name) < 0) {
	    fprintf(stderr, "Error allocating filename\n");
	    exit(1);
	}
	add_device_data(device_dir_name, device);
	
	list_add_tail(&device->devices, &devices);

	free(dir_contents[i]);
	free(device_dir_name);
    }

    free(dir_contents);
}

uint16_t file_get_random_data(int device_id, int channel_id) {
    int channel_no;
    uint16_t retval;
    random_device_obj *device;
    random_channel_obj *channel;

    list_for_each_entry(device, &devices, devices) {
	if (device->device_id != device_id) continue;

	channel_no = 0;
	list_for_each_entry(channel, &device->channels, channels) {

	    if (channel_no == channel_id) {
		retval = channel->data[channel->index++];
		if (channel->index >= channel->num_words)
		    channel->index = 0;

		return retval;
	    }

	    channel_no++;
	}
    }

    return -1;
}

void file_update_regs(uint16_t *regs, int device_id) {
    int reg_no;
    random_device_obj *device;
    random_channel_obj *channel;

    list_for_each_entry(device, &devices, devices) {

	if (device->device_id != device_id) continue;

	reg_no = device->device_id;

	list_for_each_entry(channel, &device->channels, channels) {
	    regs[reg_no++] = channel->data[channel->index++];
		if (channel->index >= channel->num_words)
		    channel->index = 0;
	}
    }
}

int file_num_devices(void) {
    return num_devices;
}

int file_get_highest_channel(void) {
    int highest = 0;

    random_device_obj *device;
    random_channel_obj *channel;

    list_for_each_entry(device, &devices, devices) {
	if (highest < device->device_id) highest = device->device_id;

	list_for_each_entry(channel, &device->channels, channels) {
	    highest++;
	}
    }

    return highest;
}

void file_print_random_data(void) {
    int i, j;
    random_device_obj *device;
    random_channel_obj *channel;

    list_for_each_entry(device, &devices, devices) {
	printf("device %i\n", device->device_id);
	i = 0;
	list_for_each_entry(channel, &device->channels, channels) {
	    printf("channel %i\n", i++);
	    for (j = 0; j < channel->num_words; j++) {
		printf("%04X ", channel->data[j]);
		if ((j % 8) == 7) printf("\n");
	    }
	    printf("\n");
	}
    }
}

void file_device_enumerate(void (*add_device_func)(int device_id)) {
    list_for_each_entry(enum_device, &devices, devices) {
	add_device_func(enum_device->device_id);
    }
    enum_device = NULL;
}

void file_channel_enumerate(
	void (*add_channel_func)(size_t num_words, uint16_t *data, void *arg),
	void *arg) {
    random_channel_obj *channel;

    if (!enum_device) return;

    list_for_each_entry(channel, &enum_device->channels, channels) {
	add_channel_func(channel->num_words, channel->data, arg);
    }
}

void file_free_random_data(void) {
    random_device_obj *device;
    random_channel_obj *channel;

    list_for_each_entry(device, &devices, devices) {
	list_for_each_entry(channel, &device->channels, channels) {
	    free(channel);
	}
	free(device);
    }

    INIT_LIST_HEAD(&devices);
    num_devices = 0;
}
