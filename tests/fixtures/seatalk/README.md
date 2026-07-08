# SeaTalk fixture captures

`.hex` files in this directory contain complete SeaTalk1 datagrams, one datagram per line, with bytes written as hexadecimal octets.

The fixture test accepts partial capture files and checks aggregate coverage across the directory. This allows small captures from individual instruments, autopilot controllers, gateways, or noisy bus segments to be added without forcing every file to contain every decoded datagram family.

When adding hardware captures, keep comments with adapter/source information and do not include raw application logs outside the SeaTalk datagram bytes.
