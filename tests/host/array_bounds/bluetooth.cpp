#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <algorithm>
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define BT_ADDR_LE_STR_LEN 30
#define LOG_ERR(...) ((void)0)
#define LOG_INF(...) ((void)0)
struct bt_addr_le_t { unsigned value; };
static size_t reported=CONFIG_BT_ID_MAX,formatted=0,calls=0;
static void bt_id_get(bt_addr_le_t *out,size_t *count) {
    ++calls;assert(out && *count==CONFIG_BT_ID_MAX);
    for(size_t i=0;i<std::min(*count,reported);++i)out[i].value=i;
    *count=reported;
}
static int bt_addr_le_to_str(const bt_addr_le_t *addr,char *,size_t) {
    assert(addr->value==formatted);++formatted;return 0;
}
#include "identity.inc"
int main() {
    assert(local_identity_addr_print()==0 && formatted==CONFIG_BT_ID_MAX && calls==1);
    formatted=0;reported=0;assert(local_identity_addr_print()==-ENOMEM && formatted==0);
    reported=CONFIG_BT_ID_MAX+1;assert(local_identity_addr_print()==-ENOMEM && formatted==0);
    std::puts("PASS: identity address logging checks returned counts and stays inside the supplied array");
}
