#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define N (8 * 1024 * 1024)

int main() {
    volatile uint64_t *arr = (volatile uint64_t *)aligned_alloc(64, N * sizeof(uint64_t));
    if (arr == NULL) {
        return 1;
    }

    uint64_t sum = 0;

    for (uint64_t i = 0; i < N; i += 8) {
        arr[i] = i;
    }

    for (uint64_t round = 0; round < 8; round++) {
        for (uint64_t i = 0; i < N; i += 8) {
            sum += arr[i];
            arr[i] = sum + i;
        }
    }

    printf("sum=%lu\n", sum);
    free((void *)arr);
    return 0;
}
