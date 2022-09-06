#include<stdio.h>

int main() {
    u_int32_t old;
    u_int32_t new;
    int32_t delta;
    int32_t frequency=400000;

    old = 3;
    new = 3;
    delta = (new - old) >= 512 ? -1*(int32_t)((old -new) & 0x3ff) : new - old ;
    frequency += delta * 10000;
    printf("old: %u\n", old);
    printf("new: %u\n", new);
    printf("delta: %d\n", delta);
    printf("frequency: %u\n", (u_int32_t)frequency);
    printf("------------------------\n");

    old = 3;
    new = 10;
    delta = (new - old) >= 512 ? -1*(int32_t)((old -new) & 0x3ff) : new - old ;
    frequency += delta * 10000;
    printf("old: %u\n", old);
    printf("new: %u\n", new);
    printf("delta: %d\n", delta);
    printf("frequency: %u\n", (u_int32_t)frequency);
    printf("------------------------\n");

    old = 3;
    new = 1019;
    delta = (new - old) >= 512 ? -1*(int32_t)((old -new) & 0x3ff) : new - old ;
    frequency += delta * 10000;
    printf("old: %u\n", old);
    printf("new: %u\n", new);
    printf("delta: %d\n", delta);
    printf("frequency: %u\n", (u_int32_t)frequency);
    printf("------------------------\n");

    old = 540;
    new = 500;
    delta = (new - old) >= 512 ? -1*(int32_t)((old -new) & 0x3ff) : new - old ;
    frequency += delta * 10000;
    printf("old: %u\n", old);
    printf("new: %u\n", new);
    printf("delta: %d\n", delta);
    printf("frequency: %u\n", (u_int32_t)frequency);
    printf("------------------------\n");

}