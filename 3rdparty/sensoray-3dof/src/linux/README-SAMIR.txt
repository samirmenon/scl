The current driver doesn't work when compiled in 64 bit mode.
On x86-64 machines, you have to compile it as a 32 bit lib.

Run commands:

"sudo apt-get install libc6-dev-i386"
"rm *.o"
"./makelib32"
"./makedemo32"



