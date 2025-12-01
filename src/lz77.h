#ifndef LZ77_H
#define LZ77_H

#include <string>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <algorithm>

// === LZ77 Compression ===
namespace lz77 {
constexpr size_t k_window_size = 2048;
constexpr size_t k_lookahead_buffer_size = 15;

struct token {
    size_t offset;
    size_t length;
    char next_char;
};

inline std::vector<token> compress_tokens(const std::string& input) {
    std::vector<token> tokens;
    size_t i = 0;
    while (i < input.size()) {
        size_t max_match_len = 0, best_offset = 0;
        size_t window_start = (i >= k_window_size) ? i - k_window_size : 0;
        for (size_t j = window_start; j < i; ++j) {
            size_t match_len = 0;
            while (match_len < k_lookahead_buffer_size &&
                   i + match_len < input.size() &&
                   input[j + match_len] == input[i + match_len]) {
                ++match_len;
            }
            if (match_len > max_match_len) {
                max_match_len = match_len;
                best_offset = i - j;
            }
        }
        char next_char = (i + max_match_len < input.size()) ? input[i + max_match_len] : '\0';
        tokens.push_back({best_offset, max_match_len, next_char});
        i += max_match_len + 1;
    }
    return tokens;
}

inline std::string decompress_tokens(const std::vector<token>& tokens) {
    std::string output;
    for (const auto& t : tokens) {
        if (t.offset > 0 && t.length > 0) {
            size_t start = output.size() - t.offset;
            for (size_t i = 0; i < t.length; ++i) {
                output.push_back(output[start + i]);
            }
        }
        if (t.next_char != '\0')
            output.push_back(t.next_char);
    }
    return output;
}

inline std::string serialize(const std::vector<token>& tokens) {
    std::string out;
    for (const auto& t : tokens) {
        out.append(reinterpret_cast<const char*>(&t.offset), sizeof(size_t));
        out.append(reinterpret_cast<const char*>(&t.length), sizeof(size_t));
        out.push_back(t.next_char);
    }
    return out;
}

inline std::vector<token> deserialize(const std::string& data) {
    std::vector<token> tokens;
    size_t i = 0;
    while (i + sizeof(size_t) * 2 + 1 <= data.size()) {
        size_t offset = *reinterpret_cast<const size_t*>(&data[i]); i += sizeof(size_t);
        size_t length = *reinterpret_cast<const size_t*>(&data[i]); i += sizeof(size_t);
        char next_char = data[i++];
        tokens.push_back({offset, length, next_char});
    }
    return tokens;
}

inline std::string compress(const std::string& input) {
    return serialize(compress_tokens(input));
}

inline std::string decompress(const std::string& input) {
    return decompress_tokens(deserialize(input));
}
} // namespace lz77

#endif // LZ77_H
