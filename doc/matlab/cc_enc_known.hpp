std::vector<uint8_t> message = {
0,0,0,0,0,0,0,1,
0,0,0,0,0,0,1,0,
0,0,0,0,0,0,1,1,
0,0,0,0,0,1,0,0,
0,0,0,0,0,1,0,1,
0,0,0,0,0,1,1,0,
0,0,0,0,0,1,1,1,
0,0,0,0,1,0,0,0,
0,0,0,0,1,0,0,1,
0,0,0,0,1,0,1,0,
0,0,0,0,1,0,1,1,
0,0,0,0,1,1,0,0,
0,0,0,0,1,1,0,1,
0,0,0,0,1,1,1,0,
0,0,0,0,1,1,1,1,
0,0,0,1,0,0,0,0,
0,0,0,1,0,0,0,1,
0,0,0,1,0,0,1,0,
0,0,0,1,0,0,1,1,
0,0,0,1,0,1,0,0,
0,0,0,1,0,1,0,1,
0,0,0,1,0,1,1,0,
0,0,0,1,0,1,1,1,
0,0,0,1,1,0,0,0,
0,0,0,1,1,0,0,1,
0,0,0,1,1,0,1,0,
0,0,0,1,1,0,1,1,
0,0,0,1,1,1,0,0,
0,0,0,1,1,1,0,1,
0,0,0,1,1,1,1,0,
0,0,0,1,1,1,1,1,
0,0,1,0,0,0,0,0,
0,0,1,0,0,0,0,1,
0,0,1,0,0,0,1,0,
0,0,1,0,0,0,1,1,
0,0,1,0,0,1,0,0,
0,0,1,0,0,1,0,1,
0,0,1,0,0,1,1,0,
0,0,1,0,0,1,1,1,
0,0,1,0,1,0,0,0,
0,0,1,0,1,0,0,1,
0,0,1,0,1,0,1,0,
0,0,1,0,1,0,1,1,
0,0,1,0,1,1,0,0,
0,0,1,0,1,1,0,1,
0,0,1,0,1,1,1,0,
0,0,1,0,1,1,1,1,
0,0,1,1,0,0,0,0,
0,0,1,1,0,0,0,1,
0,0,1,1,0,0,1,0,
0,0,1,1,0,0,1,1,
0,0,1,1,0,1,0,0,
0,0,1,1,0,1,0,1,
0,0,1,1,0,1,1,0,
0,0,1,1,0,1,1,1,
0,0,1,1,1,0,0,0,
0,0,1,1,1,0,0,1,
0,0,1,1,1,0,1,0,
0,0,1,1,1,0,1,1,
0,0,1,1,1,1,0,0,
0,0,1,1,1,1,0,1,
0,0,1,1,1,1,1,0,
0,0,1,1,1,1,1,1,
0,1,0,0,0,0,0,0,
0,1,0,0,0,0,0,1,
0,1,0,0,0,0,1,0,
0,1,0,0,0,0,1,1,
0,1,0,0,0,1,0,0,
0,1,0,0,0,1,0,1,
0,1,0,0,0,1,1,0,
0,1,0,0,0,1,1,1,
0,1,0,0,1,0,0,0,
0,1,0,0,1,0,0,1,
0,1,0,0,1,0,1,0,
0,1,0,0,1,0,1,1,
0,1,0,0,1,1,0,0,
0,1,0,0,1,1,0,1,
0,1,0,0,1,1,1,0,
0,1,0,0,1,1,1,1,
0,1,0,1,0,0,0,0,
0,1,0,1,0,0,0,1,
0,1,0,1,0,0,1,0,
0,1,0,1,0,0,1,1,
0,1,0,1,0,1,0,0,
0,1,0,1,0,1,0,1,
0,1,0,1,0,1,1,0,
0,1,0,1,0,1,1,1,
0,1,0,1,1,0,0,0,
0,1,0,1,1,0,0,1,
0,1,0,1,1,0,1,0,
0,1,0,1,1,0,1,1,
0,1,0,1,1,1,0,0,
0,1,0,1,1,1,0,1,
0,1,0,1,1,1,1,0,
0,1,0,1,1,1,1,1,
0,1,1,0,0,0,0,0,
0,1,1,0,0,0,0,1,
0,1,1,0,0,0,1,0,
0,1,1,0,0,0,1,1,
0,1,1,0,0,1,0,0,
0,1,1,0,0,1,0,1,
0,1,1,0,0,1,1,0,
0,1,1,0,0,1,1,1,
0,1,1,0,1,0,0,0,
0,1,1,0,1,0,0,1,
0,1,1,0,1,0,1,0,
0,1,1,0,1,0,1,1,
0,1,1,0,1,1,0,0,
0,1,1,0,1,1,0,1,
0,1,1,0,1,1,1,0,
0,1,1,0,1,1,1,1,
0,1,1,1,0,0,0,0,
0,1,1,1,0,0,0,1,
0,1,1,1,0,0,1,0,
0,1,1,1,0,0,1,1,
0,1,1,1,0,1,0,0,
0,1,1,1,0,1,0,1,
0,1,1,1,0,1,1,0,
0,1,1,1,0,1,1,1,
0,1,1,1,1,0,0,0,
0,1,1,1,1,0,0,1,
0,1,1,1,1,0,1,0,
0,1,1,1,1,0,1,1,
0,1,1,1,1,1,0,0,
0,1,1,1,1,1,0,1,
0,1,1,1,1,1,1,0,
0,1,1,1,1,1,1,1,
1,0,0,0,0,0,0,0,
1,0,0,0,0,0,0,1,
1,0,0,0,0,0,1,0,
1,0,0,0,0,0,1,1,
1,0,0,0,0,1,0,0,
1,0,0,0,0,1,0,1,
1,0,0,0,0,1,1,0,
1,0,0,0,0,1,1,1,
1,0,0,0,1,0,0,0,
1,0,0,0,1,0,0,1,
1,0,0,0,1,0,1,0,
1,0,0,0,1,0,1,1,
1,0,0,0,1,1,0,0,
1,0,0,0,1,1,0,1,
1,0,0,0,1,1,1,0,
1,0,0,0,1,1,1,1,
1,0,0,1,0,0,0,0,
1,0,0,1,0,0,0,1,
1,0,0,1,0,0,1,0,
1,0,0,1,0,0,1,1,
1,0,0,1,0,1,0,0,
1,0,0,1,0,1,0,1,
1,0,0,1,0,1,1,0,
1,0,0,1,0,1,1,1,
1,0,0,1,1,0,0,0,
1,0,0,1,1,0,0,1,
1,0,0,1,1,0,1,0,
1,0,0,1,1,0,1,1,
1,0,0,1,1,1,0,0,
1,0,0,1,1,1,0,1,
1,0,0,1,1,1,1,0,
1,0,0,1,1,1,1,1,
1,0,1,0,0,0,0,0,
1,0,1,0,0,0,0,1,
1,0,1,0,0,0,1,0,
1,0,1,0,0,0,1,1,
1,0,1,0,0,1,0,0,
1,0,1,0,0,1,0,1,
1,0,1,0,0,1,1,0,
1,0,1,0,0,1,1,1,
1,0,1,0,1,0,0,0,
1,0,1,0,1,0,0,1,
1,0,1,0,1,0,1,0,
1,0,1,0,1,0,1,1,
1,0,1,0,1,1,0,0,
1,0,1,0,1,1,0,1,
1,0,1,0,1,1,1,0,
1,0,1,0,1,1,1,1,
1,0,1,1,0,0,0,0,
1,0,1,1,0,0,0,1,
1,0,1,1,0,0,1,0,
1,0,1,1,0,0,1,1,
1,0,1,1,0,1,0,0,
1,0,1,1,0,1,0,1,
1,0,1,1,0,1,1,0,
1,0,1,1,0,1,1,1,
1,0,1,1,1,0,0,0,
1,0,1,1,1,0,0,1,
1,0,1,1,1,0,1,0,
1,0,1,1,1,0,1,1,
1,0,1,1,1,1,0,0,
1,0,1,1,1,1,0,1,
1,0,1,1,1,1,1,0,
1,0,1,1,1,1,1,1,
1,1,0,0,0,0,0,0,
1,1,0,0,0,0,0,1,
1,1,0,0,0,0,1,0,
1,1,0,0,0,0,1,1,
1,1,0,0,0,1,0,0,
1,1,0,0,0,1,0,1,
1,1,0,0,0,1,1,0,
1,1,0,0,0,1,1,1,
1,1,0,0,1,0,0,0,
1,1,0,0,1,0,0,1,
1,1,0,0,1,0,1,0,
1,1,0,0,1,0,1,1,
1,1,0,0,1,1,0,0,
1,1,0,0,1,1,0,1,
1,1,0,0,1,1,1,0,
1,1,0,0,1,1,1,1,
1,1,0,1,0,0,0,0,
1,1,0,1,0,0,0,1,
1,1,0,1,0,0,1,0,
1,1,0,1,0,0,1,1,
1,1,0,1,0,1,0,0,
1,1,0,1,0,1,0,1,
1,1,0,1,0,1,1,0,
1,1,0,1,0,1,1,1,
1,1,0,1,1,0,0,0,
1,1,0,1,1,0,0,1,
1,1,0,1,1,0,1,0,
1,1,0,1,1,0,1,1,
1,1,0,1,1,1,0,0,
1,1,0,1,1,1,0,1,
1,1,0,1,1,1,1,0,
1,1,0,1,1,1,1,1,
1,1,1,0,0,0,0,0,
1,1,1,0,0,0,0,1,
1,1,1,0,0,0,1,0,
1,1,1,0,0,0,1,1,
1,1,1,0,0,1,0,0,
1,1,1,0,0,1,0,1,
1,1,1,0,0,1,1,0,
1,1,1,0,0,1,1,1,
1,1,1,0,1,0,0,0,
1,1,1,0,1,0,0,1,
1,1,1,0,1,0,1,0,
1,1,1,0,1,0,1,1,
1,1,1,0,1,1,0,0,
1,1,1,0,1,1,0,1,
1,1,1,0,1,1,1,0,
1,1,1,0,1,1,1,1,
1,1,1,1,0,0,0,0,
1,1,1,1,0,0,0,1,
1,1,1,1,0,0,1,0,
1,1,1,1,0,0,1,1,
1,1,1,1,0,1,0,0,
1,1,1,1,0,1,0,1,
1,1,1,1,0,1,1,0,
1,1,1,1,0,1,1,1,
1,1,1,1,1,0,0,0,
1,1,1,1,1,0,0,1,
1,1,1,1,1,0,1,0,
1,1,1,1,1,0,1,1,
1,1,1,1,1,1,0,0,
1,1,1,1,1,1,0,1,
1,1,1,1,1,1,1,0,
1,1,1,1,1,1,1,1,
}
std::vector<uint8_t> codeword = {
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,1,1,
1,0,1,1,1,1,0,0,
0,1,1,1,1,1,1,0,
1,1,1,1,0,0,0,1,
1,1,0,0,1,1,0,1,
0,1,0,0,1,1,0,1,
1,0,0,0,1,0,1,1,
1,1,0,0,0,1,1,1,
0,0,1,1,1,0,0,0,
0,1,1,1,1,0,1,1,
0,1,0,0,0,1,0,1,
0,0,1,1,0,1,1,0,
1,1,1,1,0,1,1,0,
1,0,0,0,1,0,1,0,
0,1,0,1,1,1,1,1,
0,0,0,1,1,1,0,0,
1,1,1,0,1,1,0,0,
1,0,1,0,0,0,0,0,
1,0,0,1,0,0,0,1,
1,1,1,0,1,1,0,1,
0,0,1,0,0,0,1,0,
0,1,0,1,0,0,0,1,
0,1,1,0,0,1,0,0,
1,1,0,1,1,0,1,1,
1,1,0,1,0,1,1,1,
0,1,1,0,0,1,1,1,
1,0,1,0,1,0,1,0,
0,0,1,0,1,0,1,0,
0,0,0,1,1,0,0,1,
1,0,0,1,0,1,0,1,
0,0,0,0,1,1,0,0,
0,1,1,1,0,0,1,1,
1,0,1,1,1,1,1,1,
1,1,0,0,1,1,1,1,
1,1,0,0,0,0,1,0,
1,0,0,0,0,0,1,0,
0,1,1,1,0,0,0,1,
0,0,1,1,1,1,1,0,
0,0,1,1,0,1,1,1,
1,0,1,1,0,1,0,0,
1,0,0,0,0,1,0,0,
0,0,0,0,1,0,0,0,
1,1,1,1,1,0,0,1,
0,1,0,0,0,1,0,1,
0,1,0,0,1,0,1,0,
1,1,1,1,1,0,0,1,
1,1,1,0,0,0,1,1,
0,1,1,0,1,1,1,1,
0,1,0,1,0,0,0,0,
1,1,0,1,0,0,1,1,
0,0,1,0,1,1,0,1,
1,0,0,1,1,1,1,0,
1,0,0,1,1,1,1,0,
0,0,1,0,0,0,1,0,
1,1,0,1,1,0,0,0,
1,0,1,0,1,0,0,0,
0,1,1,0,1,0,1,1,
0,0,0,1,0,1,0,0,
0,0,0,1,0,1,1,0,
0,1,0,1,1,0,0,1,
1,0,1,0,0,1,0,1,
1,1,1,0,1,0,0,0,
0,1,0,0,0,0,0,1,
1,1,0,0,1,1,1,0,
1,1,1,1,0,0,1,0,
0,1,1,1,0,0,1,0,
1,0,0,0,1,1,1,1,
0,0,1,1,1,1,1,1,
0,0,1,1,1,1,0,0,
1,0,0,0,0,0,1,1,
0,1,1,1,1,0,1,0,
0,0,0,0,1,0,0,1,
1,1,0,0,1,0,0,1,
1,0,1,1,0,1,0,1,
1,0,1,1,0,1,0,0,
1,1,1,1,1,0,0,0,
0,0,0,0,0,1,1,1,
0,1,0,0,0,1,0,0,
1,0,1,0,1,1,1,0,
1,1,0,1,0,0,1,0,
0,0,0,1,1,1,0,1,
0,1,1,0,1,1,1,0,
0,1,1,0,0,0,0,0,
0,0,1,0,0,0,1,1,
1,1,0,1,0,0,1,1,
1,0,0,1,1,1,1,1,
1,0,0,1,0,1,0,1,
0,0,0,1,0,1,0,1,
0,0,1,0,0,1,1,0,
1,0,1,0,1,0,0,1,
0,1,0,1,1,0,1,1,
1,1,1,0,0,1,0,0,
1,1,1,0,1,0,0,0,
0,1,0,1,1,0,1,1,
1,1,1,1,1,1,0,1,
1,0,1,1,1,1,0,1,
0,1,0,0,1,1,1,0,
0,0,0,0,0,0,0,1,
0,0,1,1,0,0,1,1,
0,1,0,0,1,1,0,0,
1,0,0,0,0,0,0,0,
1,1,1,1,0,0,0,0,
1,1,0,0,0,1,1,0,
0,1,1,1,1,0,1,0,
0,1,1,1,0,1,0,1,
1,1,0,0,0,1,1,0,
0,0,0,0,1,0,0,0,
1,0,0,0,1,0,1,1,
1,0,1,1,1,0,1,1,
0,0,1,1,0,1,1,1,
0,0,0,1,0,0,1,0,
1,0,1,0,0,0,0,1,
1,0,1,0,0,0,0,1,
0,0,0,1,1,1,0,1,
1,1,0,1,1,1,0,0,
0,1,0,1,0,0,0,0,
0,1,1,0,1,1,1,1,
1,1,1,0,1,1,0,0,
0,0,1,0,1,0,0,1,
0,1,1,0,0,1,1,0,
1,0,0,1,1,0,1,0,
1,1,0,1,1,0,1,0,
1,1,1,0,0,1,1,1,
1,0,0,1,0,1,1,1,
0,1,0,1,0,1,0,0,
0,0,0,1,1,1,0,1,
0,1,1,1,0,1,1,1,
0,0,1,1,1,0,1,1,
1,1,0,0,0,1,0,0,
1,0,0,0,0,1,1,1,
1,0,1,1,1,0,0,1,
1,1,0,0,1,0,1,0,
0,0,0,0,1,0,1,0,
0,1,1,1,0,1,1,0,
0,1,0,0,1,1,0,0,
1,1,1,1,1,1,0,0,
1,1,1,1,1,1,1,1,
0,1,0,0,0,0,0,0,
1,0,0,0,0,0,1,0,
0,0,0,0,1,1,0,1,
0,0,1,1,0,0,0,1,
1,0,1,1,0,0,0,1,
1,0,0,1,1,0,0,0,
0,0,1,0,0,1,1,1,
0,0,1,0,1,0,1,1,
1,0,0,1,1,0,1,1,
0,1,0,1,0,1,1,0,
1,1,0,1,0,1,1,0,
1,1,1,0,0,1,0,1,
0,1,1,0,1,0,1,0,
1,0,1,0,0,0,1,1,
1,1,1,0,0,0,0,0,
0,0,0,1,0,0,0,0,
0,1,0,1,1,1,0,0,
0,1,1,0,1,1,0,1,
0,0,0,1,0,0,0,1,
1,1,0,1,1,1,1,0,
1,0,1,0,1,1,1,0,
1,1,0,0,1,0,1,1,
0,1,0,0,1,0,0,0,
0,1,1,1,1,0,0,0,
1,1,1,1,0,1,0,0,
0,0,0,0,0,1,0,1,
1,0,1,1,1,0,0,1,
1,0,1,1,0,1,1,0,
0,0,0,0,0,1,0,1,
1,1,1,1,0,0,0,0,
1,0,0,0,1,1,1,1,
0,1,0,0,0,0,1,1,
0,0,1,1,0,0,1,1,
0,0,1,1,1,1,1,0,
0,1,1,1,1,1,1,0,
1,0,0,0,1,1,0,1,
1,1,0,0,0,0,1,0,
0,0,1,0,0,1,0,0,
0,1,0,1,0,1,0,0,
1,0,0,1,0,1,1,1,
1,1,1,0,1,0,0,0,
1,1,1,0,1,0,1,0,
1,0,1,0,0,1,0,1,
0,1,0,1,1,0,0,1,
0,0,0,1,1,0,0,1,
0,0,0,1,1,1,1,1,
1,0,0,1,0,0,1,1,
1,0,1,0,1,1,0,0,
0,0,1,0,1,1,1,1,
1,1,0,1,0,0,0,1,
0,1,1,0,0,0,1,0,
0,1,1,0,0,0,1,0,
1,1,0,1,0,0,1,1,
1,0,0,0,0,1,1,0,
1,1,1,1,0,1,0,1,
0,0,1,1,0,1,0,1,
0,1,0,0,1,0,0,1,
0,1,0,0,1,0,0,0,
0,0,0,0,0,1,0,0,
1,1,1,1,1,0,1,1,
1,0,1,1,1,0,0,0,
1,0,1,1,1,1,0,1,
0,0,1,1,0,0,1,0,
0,0,0,0,1,1,1,0,
1,0,0,0,1,1,1,0,
0,1,1,1,0,0,1,1,
1,1,0,0,0,0,1,1,
1,1,0,0,0,0,0,0,
0,1,1,1,1,1,1,1,
0,1,1,0,1,0,0,1,
1,1,1,0,1,0,0,1,
1,1,0,1,1,0,1,0,
0,1,0,1,0,1,0,1,
1,0,1,0,0,1,1,1,
0,0,0,1,1,0,0,0,
0,0,0,1,0,1,0,0,
1,0,1,0,0,1,0,0,
0,1,0,1,0,0,1,0,
0,0,1,0,1,1,1,0,
1,1,1,0,0,0,0,1,
1,0,0,1,0,0,1,0,
1,0,0,1,1,1,0,0,
1,1,0,1,1,1,1,1,
0,0,1,0,1,1,1,1,
0,1,1,0,0,0,0,0,
0,0,1,1,1,0,1,0,
1,0,0,0,0,1,1,0,
1,0,0,0,1,0,0,1,
0,0,1,1,1,0,1,0,
1,1,1,1,0,1,0,0,
0,1,1,1,0,1,1,1,
0,1,0,0,0,1,1,1,
1,1,0,0,1,0,1,1,
0,0,0,0,0,0,0,1,
0,1,0,0,0,0,0,1,
1,0,1,1,0,0,1,0,
1,1,1,1,1,1,0,1,
1,1,0,0,1,1,1,1,
1,0,1,1,0,0,0,0,
0,1,1,1,1,1,0,0,
0,0,0,0,1,1,0,0,
1,1,0,1,0,1,0,1,
1,0,0,1,1,0,1,0,
0,1,1,0,0,1,1,0,
0,0,1,0,0,1,1,0,
0,0,0,1,1,0,1,1,
0,1,1,0,1,0,1,1,
1,0,1,0,1,0,0,0,
1,1,0,1,0,1,1,1,
1,1,1,0,1,1,1,0,
0,1,0,1,1,1,0,1,
0,1,0,1,1,1,0,1,
1,1,1,0,0,0,0,1,
0,0,1,0,0,0,0,0,
1,0,1,0,1,1,0,0,
1,0,0,1,0,0,1,1,
1,1,0,0,1,0,0,1,
1,0,1,0,1,1,0,0,
1,1,1,0,1,1,1,1,
0,0,0,1,1,1,1,1,
0,1,0,1,0,0,1,1,
0,1,1,0,0,0,1,0,
0,0,0,1,1,1,1,0,
1,1,0,1,0,0,0,1,
1,0,1,0,0,0,1,0,
1,0,0,1,0,1,1,1,
0,0,1,0,1,0,0,0,
0,0,1,0,0,1,0,0,
1,0,0,1,0,1,0,0,
0,1,0,1,1,0,0,1,
1,1,0,1,1,0,0,1,
1,1,1,0,1,0,1,0,
0,1,1,0,0,1,0,1,
0,1,0,0,0,0,1,1,
1,1,1,1,0,0,1,1,
1,1,1,1,0,0,0,0,
0,1,0,0,1,1,1,1,
1,0,0,0,1,1,0,1,
0,0,0,0,0,0,1,0,
0,0,1,1,1,1,1,0,
1,0,1,1,1,1,1,0,
0,1,1,1,1,0,0,0,
0,0,1,1,0,1,0,0,
1,1,0,0,1,0,1,1,
1,0,0,0,1,0,0,0,
1,0,1,1,0,1,1,0,
1,1,0,0,0,1,0,1,
0,0,0,0,0,1,0,1,
0,1,1,1,1,0,1,0,
0,0,0,1,0,0,0,0,
1,0,0,1,1,1,0,0,
1,0,1,0,0,0,1,1,
0,0,1,0,0,0,0,0,
1,1,0,1,1,1,1,0,
0,1,1,0,1,1,0,1,
0,1,1,0,1,1,0,1,
1,1,0,1,0,0,0,1,
0,0,1,0,1,0,1,1,
0,1,0,1,1,0,1,1,
1,0,0,1,1,0,0,0,
1,1,1,0,0,1,1,1,
1,1,1,0,0,1,0,1,
1,0,1,0,1,0,1,0,
0,1,0,1,0,1,1,0,
0,0,0,1,0,1,1,0,
1,1,1,1,1,1,1,1,
1,0,0,0,0,0,0,0,
0,1,0,0,1,1,0,0,
0,0,1,1,1,1,0,0,
0,0,1,1,0,0,0,1,
0,1,1,1,0,0,0,1,
1,0,0,0,0,0,1,0,
1,1,0,0,1,1,0,1,
1,1,0,0,0,1,0,0,
0,1,0,0,0,1,1,1,
0,1,1,1,0,1,1,1,
1,1,1,1,1,0,1,1,
0,0,0,0,1,0,1,0,
1,0,1,1,0,1,1,0,
1,0,1,1,1,0,0,1,
0,0,0,0,0,1,1,1,
0,1,0,1,1,1,0,1,
0,0,1,0,0,0,0,1,
1,1,1,0,1,1,1,0,
1,0,0,1,1,1,0,1,
1,0,0,1,0,0,1,1,
1,1,0,1,0,0,0,0,
0,0,1,0,0,0,0,0,
0,1,1,0,1,1,0,0,
0,1,1,0,0,1,1,0,
1,1,1,0,0,1,1,0,
1,1,0,1,0,1,0,1,
0,1,0,1,1,0,1,0,
1,0,1,0,1,0,0,0,
0,0,0,1,0,1,1,1,
0,0,0,1,1,0,1,1,
1,0,1,0,1,0,1,1,
1,0,1,1,0,0,1,0,
0,0,1,1,1,1,0,1,
0,0,0,0,0,0,0,1,
1,0,0,0,0,0,0,1,
0,1,1,1,1,1,0,0,
1,1,0,0,1,1,0,0,
1,1,0,0,1,1,1,1,
0,1,1,1,0,0,0,0,
1,0,0,0,1,0,0,1,
1,1,1,1,1,0,1,0,
0,0,1,1,1,0,1,0,
0,1,0,0,0,1,1,0,
0,1,0,0,0,1,1,1,
0,0,0,0,1,0,1,1,
1,1,1,1,0,1,0,0,
1,0,1,1,0,1,0,0,
1,1,1,0,0,0,0,1,
0,1,0,1,0,0,1,0,
0,1,0,1,0,0,1,0,
1,1,1,0,1,1,1,0,
0,0,1,0,1,1,1,1,
1,0,1,0,0,0,1,1,
1,0,0,1,1,1,0,0,
0,0,0,1,1,1,1,1,
1,1,0,1,1,0,1,0,
1,0,0,1,0,1,0,1,
0,1,1,0,1,0,0,1,
0,0,1,0,1,0,0,1,
0,0,0,1,0,1,0,0,
0,1,1,0,0,1,0,0,
1,0,1,0,0,1,1,1,
1,1,0,1,1,0,0,0,
0,0,0,0,1,1,1,0,
0,1,0,0,1,1,1,0,
1,0,1,1,1,1,0,1,
1,1,1,1,0,0,1,0,
1,1,0,0,0,0,0,0,
1,0,1,1,1,1,1,1,
0,1,1,1,0,0,1,1,
0,0,0,0,0,0,1,1,
0,0,1,1,0,1,0,1,
1,0,0,0,1,0,0,1,
1,0,0,0,0,1,1,0,
0,0,1,1,0,1,0,1,
1,1,1,1,1,0,1,1,
0,1,1,1,1,0,0,0,
0,1,0,0,1,0,0,0,
1,1,1,1,0,0,1,0,
0,1,1,0,1,0,1,1,
1,1,0,1,0,1,0,0,
1,1,0,1,1,0,0,0,
0,1,1,0,1,0,0,0,
1,0,1,0,0,1,0,1,
0,0,1,0,0,1,0,1,
0,0,0,1,0,1,1,0,
1,0,0,1,1,0,0,1,
0,1,0,1,0,0,0,0,
0,0,0,1,0,0,1,1,
1,1,1,0,0,0,1,1,
1,0,1,0,1,1,1,1,
1,0,0,1,1,1,1,0,
1,1,1,0,0,0,1,0,
0,0,1,0,1,1,0,1,
0,1,0,1,1,1,1,0,
1,0,0,0,0,1,0,0,
1,1,0,0,1,0,0,0,
0,0,1,1,0,1,1,1,
0,1,1,1,0,1,0,0,
0,1,0,0,1,0,1,0,
0,0,1,1,1,0,0,1,
1,1,1,1,1,0,0,1,
1,0,0,0,0,1,0,1,
1,0,1,1,1,1,1,1,
0,0,0,0,1,1,1,1,
0,0,0,0,1,1,0,0,
1,0,1,1,0,0,1,1,
0,1,1,1,0,0,0,1,
1,1,1,1,1,1,1,0,
1,1,0,0,0,0,1,0,
0,1,0,0,0,0,0,1,
1,1,0,1,0,1,1,1,
1,0,1,0,0,1,1,1,
0,1,1,0,0,1,0,0,
0,0,0,1,1,0,1,1,
0,0,0,1,1,0,0,1,
0,1,0,1,0,1,1,0,
1,0,1,0,1,0,1,0,
1,1,1,0,1,0,1,0,
1,1,1,0,1,1,0,0,
0,1,1,0,0,0,0,0,
0,1,0,1,1,1,1,1,
1,1,0,1,1,1,0,0,
0,0,1,0,0,0,1,0,
1,0,0,1,0,0,0,1,
1,0,0,1,0,0,0,1,
0,0,1,0,1,1,0,1,
0,0,1,1,1,0,0,0,
1,0,1,1,1,0,1,1,
1,0,0,0,1,0,1,1,
0,0,0,0,0,1,1,1,
1,1,1,1,0,1,1,0,
0,1,0,0,1,0,1,0,
0,1,0,0,0,1,0,1,
1,1,1,1,0,1,1,0,
0,0,0,0,0,0,1,1,
0,1,1,1,1,1,0,0,
1,0,1,1,0,0,0,0,
1,1,0,0,0,0,0,0,
1,1,0,0,1,1,0,1,
1,0,0,0,1,1,0,1,
0,1,1,1,1,1,1,0,
0,0,1,1,1,1,0,0,
1,0,0,1,1,0,1,0,
0,0,0,1,1,0,1,0,
0,0,1,0,1,0,0,1,
1,0,1,0,0,1,1,0,
0,1,0,1,0,1,0,0,
1,1,1,0,1,0,1,1,
1,1,1,0,0,1,1,1,
0,1,0,1,0,1,1,1,
1,0,1,0,0,0,0,1,
1,1,0,1,1,1,0,1,
0,0,0,1,0,0,1,0,
0,1,1,0,0,0,0,1,
0,1,1,0,1,1,1,1,
0,0,1,0,1,1,0,0,
1,1,0,1,1,1,0,0,
1,0,0,1,0,0,0,0,
0,1,1,1,0,1,0,1,
0,0,0,0,0,1,1,0,
1,1,0,0,0,1,1,0,
1,0,1,1,1,0,1,0,
1,0,1,1,1,0,1,1,
1,1,1,1,0,1,1,1,
0,0,0,0,1,0,0,0,
0,1,0,0,1,0,1,1,
0,1,0,0,1,1,1,0,
1,1,0,0,0,0,0,1,
1,1,1,1,1,1,0,1,
0,1,1,1,1,1,0,1,
1,0,0,0,0,0,0,0,
0,0,1,1,0,0,0,0,
0,0,1,1,0,0,1,1,
1,0,0,0,1,1,1,1,
0,0,1,0,0,1,1,0,
0,1,1,0,1,0,0,1,
1,0,0,1,0,1,0,1,
1,1,0,1,0,1,0,1,
1,1,1,0,1,0,0,0,
1,0,0,1,1,0,0,0,
0,1,0,1,1,0,1,1,
0,0,1,0,0,1,0,0,
0,0,0,1,1,1,0,1,
1,0,1,0,1,1,1,0,
1,0,1,0,1,1,1,0,
0,0,0,1,0,0,1,0,
1,1,0,1,0,0,1,1,
0,1,0,1,1,1,1,1,
0,1,1,0,0,0,0,0,
1,1,1,0,0,0,1,1,
1,1,0,0,1,0,0,1,
0,1,1,1,0,1,0,1,
0,1,1,1,1,0,1,0,
1,1,0,0,1,0,0,1,
0,0,0,0,0,1,1,1,
1,0,0,0,0,1,0,0,
1,0,1,1,0,1,0,0,
0,0,1,1,1,0,0,0,
1,1,1,1,0,0,1,0,
1,0,1,1,0,0,1,0,
0,1,0,0,0,0,0,1,
0,0,0,0,1,1,1,0,
0,0,1,1,1,1,0,0,
0,1,0,0,0,0,1,1,
1,0,0,0,1,1,1,1,
}
