#ifndef ZBASE64_H
#define ZBASE64_H

#include <string>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <algorithm>

namespace zbase64 {

const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

inline bool is_base64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/') || (c == '='));
}

inline std::string encode(const std::string& input) {
    size_t in_len = input.size();
    size_t i = 0, j = 0;
    unsigned char char_array_3[3], char_array_4[4];
    std::string ret;

    while (in_len--) {
        char_array_3[i++] = input[j++];
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) | ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) | ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i < 4); i++) {
                ret += base64_chars[char_array_4[i]];
            }
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 3; j++) {
            char_array_3[j] = '\0';
        }
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) | ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) | ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++) {
            ret += base64_chars[char_array_4[j]];
        }

        while ((i++ < 3)) {
            ret += '=';
        }
    }

    return ret;
}

inline std::string decode(const std::string& input) {
    size_t in_len = input.size();
    size_t i = 0, j = 0, in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::string ret;

    while (in_len-- && (input[in_] != '=') && is_base64(input[in_])) {
        char_array_4[i++] = input[in_];
        in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++) {
                char_array_4[i] = base64_chars.find(char_array_4[i]);
            }

            char_array_3[0] = (char_array_4[0] << 2) | (char_array_4[1] >> 4);
            char_array_3[1] = ((char_array_4[1] & 0x0f) << 4) | (char_array_4[2] >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x03) << 6) | char_array_4[3];

            for (i = 0; (i < 3); i++) {
                ret += char_array_3[i];
            }
            i = 0;
        }
    }

    // 收尾处理  增加内容
    if (i) {
        for (size_t j = i; j < 4; j++)
            char_array_4[j] = 0;

        for (size_t j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) | (char_array_4[1] >> 4);
        char_array_3[1] = ((char_array_4[1] & 0x0f) << 4) | (char_array_4[2] >> 2);

        for (size_t j = 0; j < (i - 1); j++)
            ret += char_array_3[j];
    }

    return ret;
}

} // namespace base64
#endif // BASE64_H
