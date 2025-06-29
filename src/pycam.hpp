#ifndef PYCAM_H
#define PYCAM_H

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

    linked_item(linked_list<T> *list, T si, linked_item *prev, linked_item *next)
        : list(list), si(si), prevItem(prev), nextItem(next) {}

    void remove() {
        if (prevItem) prevItem->nextItem = nextItem;
        else list->head = nextItem;

        if (nextItem) nextItem->prevItem = prevItem;
        else list->tail = prevItem;

        delete this;
    }

    void insert_after(T new_si) {
        linked_item *new_item = new linked_item(list, new_si, this, nextItem);
        if (nextItem) nextItem->prevItem = new_item;
        else list->tail = new_item;
        nextItem = new_item;
    }

    void insert_before(T new_si) {
        linked_item *new_item = new linked_item(list, new_si, prevItem, this);
        if (prevItem) prevItem->nextItem = new_item;
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

    size_t size() {
        linked_item<T> *current = head;
        size_t count = 0;
        while (current) {
            count++;
            current = current->nextItem;
        }
        return count;
    }

    void add(T si) {
        if (!tail) {
            head = tail = new linked_item{this, si, (linked_item<T>*)nullptr, (linked_item<T>*)nullptr};
        } else {
            tail->nextItem = new linked_item{this, si, tail, (linked_item<T>*)nullptr};
            tail = tail->nextItem;
        }
    }
};


#endif