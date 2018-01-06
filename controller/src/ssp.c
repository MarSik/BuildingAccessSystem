/* Definitions for stack protector */

int __stack_chk_guard = 0xdeadbeef;

void __stack_chk_fail(void)
{
    while (1) {
    }
}

