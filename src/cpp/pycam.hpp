#ifndef PYCAM_H
#define PYCAM_H

#include <cstddef>

#define ANGULAR_DEG_TOL 0.05
#define ANGULAR_RAD_TOL 1e-3
#define ANGULAR_CROSS_TOL 1e-3
#define POS_1D_TOL 1e-6
#define POS_2D_TOL 1e-6
#define POS_2D_TOL_SQUARED 1e-12

// Variable stack-allocated array
template <typename T, size_t N>
struct varray {
    T data[N];
    size_t count;
};

template <typename T>
struct result {
    T data;
    bool success;
};

template <typename T>
struct linked_list;

template <typename T>
struct linked_item {
    T si;
    linked_item *prevItem;
    linked_item *nextItem;
    linked_list<T> *list = nullptr;

    linked_item(linked_list<T> *list, linked_item *prev, linked_item *next)
        : list(list), prevItem(prev), nextItem(next) {}

    linked_item(linked_list<T> *list, T si, linked_item *prev, linked_item *next)
        : list(list), si(si), prevItem(prev), nextItem(next) {}

    void remove() {
        if (prevItem != nullptr) prevItem->nextItem = nextItem;
        else list->head = nextItem;

        if (nextItem != nullptr) nextItem->prevItem = prevItem;
        else list->tail = prevItem;

        delete this;
    }

    void insert_after(T new_si) {
        linked_item *new_item = new linked_item(list, new_si, this, nextItem);
        if (nextItem != nullptr) nextItem->prevItem = new_item;
        else list->tail = new_item;
        nextItem = new_item;
    }

    void insert_before(T new_si) {
        linked_item *new_item = new linked_item(list, new_si, prevItem, this);
        if (prevItem != nullptr) prevItem->nextItem = new_item;
        else list->head = new_item;
        prevItem = new_item;
    }
};

template <typename T>
struct linked_list {
    linked_item<T> *head;
    linked_item<T> *tail;

    linked_list() : head(nullptr), tail(nullptr) {}

    ~linked_list() {
        while (head) {
            linked_item<T> *next = head->nextItem;
            delete head;
            head = next;
        }
        tail = nullptr;
    }

    size_t size() const;

    linked_item<T>* add() {
        if (!tail) {
            head = tail = new linked_item<T>{this, (linked_item<T>*)nullptr, (linked_item<T>*)nullptr};
        } else {
            tail->nextItem = new linked_item<T>{this, tail, (linked_item<T>*)nullptr};
            tail = tail->nextItem;
        }
        return tail;
    }

    linked_item<T>* add(T si) {
        if (!tail) {
            head = tail = new linked_item<T>{this, si, (linked_item<T>*)nullptr, (linked_item<T>*)nullptr};
        } else {
            tail->nextItem = new linked_item<T>{this, si, tail, (linked_item<T>*)nullptr};
            tail = tail->nextItem;
        }
        return tail;
    }
};


#endif