target remote :3333

define reset
    load
    set $pc = *(unsigned int *) 0x8000004
end
