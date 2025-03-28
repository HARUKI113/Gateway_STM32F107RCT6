/*
 * basic_queue.h
 *
 *  Created on: Mar 22, 2021
 *      Author: robokenpckairo
 */

#ifndef INC_BASIC_QUEUE_H_
#define INC_BASIC_QUEUE_H_

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif

struct BasicQueue_t
{
	size_t next_write;
	size_t next_read;
	size_t count;
	size_t max_count;

	size_t data_size;
	uint8_t* ring_buffer;
};


static inline void BasicQueue_clear(struct BasicQueue_t* obj)
{
	obj->next_write = 0;
	obj->next_read = 0;
	obj->count = 0;
	memset(obj->ring_buffer, 0x00, obj->data_size * obj->max_count);
}


static inline void BasicQueue_init(struct BasicQueue_t* obj, size_t data_size, size_t max_count, void* buffer)
{
	obj->data_size = data_size;

	obj->max_count = max_count;
	obj->ring_buffer = (uint8_t*)buffer;

	BasicQueue_clear(obj);
}


static inline size_t BasicQueue_count(struct BasicQueue_t* obj)
{
	return obj->count;
}


static inline size_t BasicQueue_isFill(struct BasicQueue_t* obj)
{
	return obj->count == obj->max_count;
}


static inline size_t BasicQueue_maxCount(struct BasicQueue_t* obj)
{
    return obj->max_count;
}


static inline size_t BasicQueue_dataSize(struct BasicQueue_t* obj)
{
    return obj->data_size;
}

static inline void BasicQueue_add(struct BasicQueue_t* obj, const void* data)
{
	if(BasicQueue_isFill(obj))
	{
		obj->count--;
		obj->next_read = (obj->next_read + 1) % obj->max_count;
	}

	memcpy(&obj->ring_buffer[obj->next_write * obj->data_size], data, obj->data_size);
	obj->count++;
    obj->next_write = (obj->next_write + 1) % obj->max_count;
}


static inline int BasicQueue_readonly(struct BasicQueue_t* obj, void* data, size_t index)
{
	if(obj->count <= index)
	{
		return 0;
	}else{
        index = (obj->next_read + index) % obj->max_count;
		memcpy(data, &obj->ring_buffer[obj->next_read * obj->data_size], obj->data_size);
		return 1;
	}
}

static inline int BasicQueue_get(struct BasicQueue_t* obj, void* data)
{
	if(obj->count == 0)
	{
		return 0;
	}else{
		memcpy(data, &obj->ring_buffer[obj->next_read * obj->data_size], obj->data_size);
		obj->count--;
        obj->next_read = (obj->next_read + 1) % obj->max_count;
		return 1;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* INC_BASIC_QUEUE_H_ */
