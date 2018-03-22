#Getting started

To run the tests:
```
rake test:all
```

To build and deploy to a board:
```
rake release
arm-none-eabi-gdb
```

If things don't appear to be compiling and you've changed a #define or a header file...
```
rake clobber
rake release
arm-none-eabi-gdb
```


# Building a bootloader+firmware image
###Command
```
./firmware_sandwich_toaster.sh
```

###Explanation
This clobbers, rake release's stroller-x-bt, rake release's stx-bootloader, combines them, and burns that combined binary to the board

###Dependencies:

Assuming your current folder is:

`~/gitlab/4moms/embedded/stroller-x-bt`

you must also have

`~/gitlab/4moms/embedded/stx-bootloader`

` ~/gitlab/4moms/embedded/firmware_creator`
