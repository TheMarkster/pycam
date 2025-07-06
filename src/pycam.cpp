#include "pycam.hpp"
#include "intersection.hpp"
#include <cmath>

template <typename T>
size_t linked_list<T>::size() const {
    linked_item<T> *current = head;
    size_t count = 0;
    while (current) {
        count++;
        current = current->nextItem;
    }
    return count;
}

// Explicit template instantiation for segment_info
template class linked_list<segment_info>;
template class linked_item<segment_info>;