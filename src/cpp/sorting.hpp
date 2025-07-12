#include <iostream>
#include <vector>

template <typename T, typename U>
struct sort_item {
    T value;
    U* data;
};

template <typename T, typename U>
void merge(std::vector<sort_item<T, U>>& items, int left, int mid, int right) {
    int n1 = mid - left + 1;
    int n2 = right - mid;

    std::vector<sort_item<T, U>> L(n1);
    std::vector<sort_item<T, U>> R(n2);

    for (int i = 0; i < n1; ++i)
        L[i] = items[left + i];
    for (int j = 0; j < n2; ++j)
        R[j] = items[mid + 1 + j];

    int i = 0, j = 0, k = left;
    while (i < n1 && j < n2) {
        if (L[i].value <= R[j].value)
            items[k++] = L[i++];
        else
            items[k++] = R[j++];
    }

    while (i < n1) items[k++] = L[i++];
    while (j < n2) items[k++] = R[j++];
}

template <typename T, typename U>
void merge_sort(std::vector<sort_item<T, U>>& items, int left, int right) {
    if (left < right) {
        int mid = left + (right - left) / 2;
        merge_sort(items, left, mid);
        merge_sort(items, mid + 1, right);
        merge(items, left, mid, right);
    }
}
