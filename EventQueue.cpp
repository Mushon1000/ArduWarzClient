/*
* EventQueue.cpp
*
* Part of Arduino Event System.
*
* Author: mromani@ottotecnica.com
* Copyright (c) 2010 OTTOTECNICA Italy
*
* This library is free software; you can redistribute it
* and/or modify it under the terms of the GNU Lesser
* General Public License as published by the Free Software
* Foundation; either version 2.1 of the License, or (at
* your option) any later version.
*
* This library is distributed in the hope that it will
* be useful, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A
* PARTICULAR PURPOSE.  See the GNU Lesser General Public
* License for more details.
*
* You should have received a copy of the GNU Lesser
* General Public License along with this library; if not,
* write to the Free Software Foundation, Inc.,
* 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*
*/

#include "EventQueue.h"


EventQueue::EventQueue() {
	init();
}


void EventQueue::init() {
	int i;
	eventQueueHead = 0;
	eventQueueTail = EVQUEUE_SIZE - 1;
	numEvents = 0;

	for (i = 0; i < EVQUEUE_SIZE; i++) {
		eventQueue[i] = 0;
		eventParam[i] = 0;
		eventParam2[i] = 0;

	}
}


boolean EventQueue::isEmpty() {
	return (numEvents == 0);
}


boolean EventQueue::isFull() {
	return (eventQueueHead == eventQueueTail);
}


int EventQueue::getNumEvents() {
	return numEvents;
}


boolean EventQueue::enqueueEvent(int ev_code, int ev_param, int ev_param2) {

	if (isFull()) {
		// log the queue full error
		Serial.print(millis());
		Serial.println(" QUEUE FULL");
		return false;
	}

	// store the event
	eventQueue[eventQueueHead] = ev_code;
	eventParam[eventQueueHead] = ev_param;
	eventParam2[eventQueueHead] = ev_param2;


	// update queue head value
	eventQueueHead = (eventQueueHead + 1) % EVQUEUE_SIZE;;

	// update number of events in queue
	//numEvents++;
	numEvents = numEvents + 1;
	//delay(10);
	//Serial.print("enq:");
	//Serial.println(numEvents);
	return true;
}


boolean EventQueue::dequeueEvent(int* ev_code, int* ev_param, int* ev_param2) {
	//int temp;
	//boolean isEmpty;
	//delay(10);
	//Serial.print("deq direct:");
	//Serial.println(numEvents);

	//Serial.print("deq is empty as get:");
	//Serial.println(EventQueue::isEmpty());
	//numEvents = 1;
	if (numEvents == 0) {
		
		return false;
	}
	//delay(10);
	//Serial.println("...");
	eventQueueTail = (eventQueueTail + 1) % EVQUEUE_SIZE;

	// store event code and event parameter
	// into the user-supplied variables
	*ev_code = eventQueue[eventQueueTail];
	*ev_param = eventParam[eventQueueTail];
	*ev_param2 = eventParam2[eventQueueTail];

	// update number of events in queue
	numEvents--;
	//Serial.print("deq:");
	//Serial.println(numEvents);
	return true;
}
