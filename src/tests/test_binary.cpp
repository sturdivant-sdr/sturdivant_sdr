#include <bitset>
#include <iostream>

int main() {
    uint8_t val1 = 0b10000001;
    uint8_t val2 = 0b10001000;
    uint8_t b = 0b00000001;
    std::bitset<8> out;

    out = val1;
    std::cout << "val1: {" << out << "}" << std::endl;
    out = val2;
    std::cout << "val2: {" << out << "}" << std::endl << std::endl;
    out = b;
    std::cout << "b: {" << out << "}" << std::endl << std::endl;

    //* BITWISE AND
    out = val1 & val2;
    std::cout << "BITWISE AND - val1 & val2: {" << out << "}" << std::endl;

    //* BITWISE OR
    out = val1 | val2;
    std::cout << "BITWISE OR  - val1 | val2: {" << out << "}" << std::endl;

    //* BITWISE XOR
    out = val1 ^ val2;
    std::cout << "BITWISE XOR - val1 ^ val2: {" << out << "}" << std::endl;

    //* BITWISE NOT
    out = ~val1;
    std::cout << "BITWISE NOT - ~val1: {" << out << "}" << std::endl;

    //* LEFT SHIFT
    out = val1 << 3;
    std::cout << "LEFT SHIFT  - val1 << 3: {" << out << "}" << std::endl;

    //* RIGHT SHIFT
    out = val1 >> 3;
    std::cout << "RIGHT SHIFT - val1 >> 3: {" << out << "}" << std::endl;

    //* TEST 1 (getting value of bit)
    std::cout << std::endl << "Getting value of bits ... " << std::endl;
    out = (val2 >> 2);
    std::cout << "TEST 1a: val2 >> 2: {" << out << "}" << std::endl;
    out = (val2 >> 2) & 1;
    std::cout << "TEST 1a: (val2 >> 2) & 1: {" << out << "}" << std::endl;
    out = (val2 >> 3);
    std::cout << "TEST 1b: val2 >> 3: {" << out << "}" << std::endl;
    out = (val2 >> 3) & 1;
    std::cout << "TEST 1b: (val2 >> 3) & 1: {" << out << "}" << std::endl;

    //* TEST 2 (setting value of bit)
    std::cout << std::endl << "Setting value of bits to 1 ... " << std::endl;
    out = val2 | (1 << 3);
    std::cout << "TEST 2a: val2 | (1 << 3): {" << out << "}" << std::endl;
    out = val2 | (1 << 4);
    std::cout << "TEST 2b: val2 | (1 << 4): {" << out << "}" << std::endl;

    //* TEST 3
    std::cout << std::endl;
    out = -b;
    std::cout << "-b: {" << out << "} = " << out << std::endl;

    out = (-b ^ val2);
    std::cout << "-b ^ val2: {" << out << "} = " << out << std::endl;

    out = (-b ^ val2) & (1 << 2);
    std::cout << "(-b ^ val2) & (1 << 2): {" << out << "} = " << out << std::endl;

    // std::cout << std::endl;
    // uint32_t a = 0b00000111111110000000000000000000;
    // uint8_t aa = (a >> 19) & ((1 << 9) - 1);
    // std::bitset<32> out2;
    // // uint32_t tmp = a;
    // out2 = a;
    // std::cout << "a: {" << out2 << "} = " << (int)a << std::endl;
    // // tmp >>= (31 - 12);
    // // out2 = tmp;
    // // std::cout << "tmp: {" << out2 << "} = " << (int)tmp << std::endl;
    // // tmp &= (1 << (12 - 4 + 1)) - 1;
    // // out2 = tmp;
    // // std::cout << "tmp: {" << out2 << "} = " << (int)tmp << std::endl;
    // // aa = tmp;
    // out = aa;
    // out2 = ((1 << 9) - 1);
    // std::cout << "(a >> 19) & ((1 << 9) - 1): {" << out << "} = " << (int)aa << std::endl;
    // std::cout << out2 << std::endl;

    std::cout << std::endl;
    std::bitset<64> out2;
    uint64_t z = 129;
    uint64_t zz = z;
    out2 = z;
    std::cout << "z: {" << out2 << "} = " << (int)z << std::endl;
    zz &= ((1 << 8) - 1);
    out2 = zz;
    std::cout << "zz: {" << out2 << "} = " << (int)z << std::endl;
    uint64_t sb = 1 << 7;  // signed bit
    out2 = sb;
    std::cout << "sb: {" << out2 << "} = " << (int)sb << std::endl;
    out2 = zz & ~sb;
    std::cout << "zz & ~sb: {" << out2 << "} = " << (int)(zz & ~sb) << std::endl;
    out2 = (sb - (zz & ~sb));
    std::cout << "sb - (zz & ~b)): {" << out2 << "} = " << (double)(sb - (zz & ~sb)) << std::endl;
    std::cout << "true answer = " << -(double)(sb - (zz & ~sb)) << std::endl;
    // std::cout << "zz: {" << -static_cast<double>(zz) << "}" << std::endl;
    // std::cout << "zz: {" << static_cast<double>(z) - std::pow(2.0, 8.0) << "}" << std::endl;
}