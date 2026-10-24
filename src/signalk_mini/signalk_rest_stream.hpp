#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace signalk_mini {

// Resumable REST responses: when a response does not fit the client's
// transmit buffer, it is regenerated from the typed model on each pass and
// the bytes already sent are skipped. Resume points are leaf JSON values
// (numbers, strings, true/false/null), so values that changed between passes
// are simply sent as their newer value. A CRC over the document structure
// (punctuation and object keys) of the skipped part detects a structure
// change (a value appearing or expiring); the response is then aborted rather
// than sending invalid JSON.

inline uint32_t crc32_update(uint32_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    return crc;
}

// Classifies compact JSON one byte at a time.
class JsonLeafLexer {
public:
    struct Step {
        bool leaf_ends_before = false; // a number/literal ended just before this byte
        bool leaf_ends_after = false;  // this byte closed a string value
        bool structural = false;       // punctuation or part of an object key
    };

    Step feed(char c) {
        Step step;
        if (in_string_) {
            step.structural = string_is_key_;
            if (escape_) {
                escape_ = false;
            } else if (c == '\\') {
                escape_ = true;
            } else if (c == '"') {
                in_string_ = false;
                if (!string_is_key_) step.leaf_ends_after = true;
            }
            return step;
        }
        if (in_scalar_) {
            if (c != ',' && c != '}' && c != ']') return step;
            in_scalar_ = false;
            step.leaf_ends_before = true;
        }
        switch (c) {
        case '{':
            push(true);
            expect_key_ = true;
            step.structural = true;
            break;
        case '[':
            push(false);
            expect_key_ = false;
            step.structural = true;
            break;
        case '}':
        case ']':
            pop();
            expect_key_ = false;
            step.structural = true;
            break;
        case ',':
            expect_key_ = top_is_object();
            step.structural = true;
            break;
        case ':':
            expect_key_ = false;
            step.structural = true;
            break;
        case '"':
            in_string_ = true;
            string_is_key_ = expect_key_ && top_is_object();
            step.structural = string_is_key_;
            break;
        default:
            in_scalar_ = true;
            break;
        }
        return step;
    }

    // A document that ends with a bare number or literal.
    bool finish_leaf() {
        const bool ended = in_scalar_;
        in_scalar_ = false;
        return ended;
    }

private:
    void push(bool object) {
        if (depth_ < 32) objects_ = object ? (objects_ | (1u << depth_)) : (objects_ & ~(1u << depth_));
        ++depth_;
    }
    void pop() {
        if (depth_ > 0) --depth_;
    }
    bool top_is_object() const { return depth_ > 0 && depth_ <= 32 && (objects_ & (1u << (depth_ - 1))) != 0; }

    uint32_t objects_ = 0;
    uint8_t depth_ = 0;
    bool in_string_ = false;
    bool escape_ = false;
    bool string_is_key_ = false;
    bool in_scalar_ = false;
    bool expect_key_ = false;
};

struct RestResumeJob {
    static constexpr size_t MaxPathLength = 160;

    void* connection = nullptr;
    char path[MaxPathLength]{};
    uint64_t now_us = 0;
    uint32_t leaves_sent = 0;
    uint32_t structure_crc = 0;
    uint32_t last_progress_ms = 0;
    bool header_sent = false;
    bool active() const { return connection != nullptr; }
};

enum class RestPassResult : uint8_t { Complete, Paused, Failed };

// One regeneration pass. Bytes up to `job.leaves_sent` leaves are skipped and
// checked against the stored structure CRC; after that, bytes are collected
// per leaf and written only when the whole leaf segment fits `room()`.
template<typename Write, typename Room>
class RestResumeSink {
public:
    static constexpr size_t SegmentCapacity = 512;

    RestResumeSink(RestResumeJob& job, Write write, Room room) : job_(job), write_(write), room_(room) {}

    static bool write(void* context, const char* data, size_t len) {
        auto* self = static_cast<RestResumeSink*>(context);
        for (size_t i = 0; i < len; ++i) {
            if (!self->feed(data[i])) return false;
        }
        return true;
    }

    // Call once the writer finished the whole document.
    RestPassResult finish() {
        if (result_ != RestPassResult::Complete) return result_;
        if (lexer_.finish_leaf() && !boundary()) return result_;
        // The regenerated document ended before the part already sent: its
        // structure changed and the sent prefix cannot be completed.
        if (leaf_index_ < job_.leaves_sent) return RestPassResult::Failed;
        if (!commit()) return result_;
        return RestPassResult::Complete;
    }

    RestPassResult result() const { return result_; }

private:
    bool feed(char c) {
        const JsonLeafLexer::Step step = lexer_.feed(c);
        if (step.leaf_ends_before && !boundary()) return false;
        if (leaf_index_ >= job_.leaves_sent && !append(c)) return false;
        if (step.structural) crc_ = crc32_update(crc_, static_cast<uint8_t>(c));
        if (step.leaf_ends_after && !boundary()) return false;
        return true;
    }

    bool boundary() {
        ++leaf_index_;
        if (leaf_index_ == job_.leaves_sent) {
            if (crc_ != job_.structure_crc) return fail();
            return true;
        }
        if (leaf_index_ < job_.leaves_sent) return true;
        if (!commit()) return false;
        job_.leaves_sent = leaf_index_;
        job_.structure_crc = crc_;
        return true;
    }

    bool append(char c) {
        if (used_ == sizeof(segment_)) {
            // A single leaf larger than the segment: send it in pieces if the
            // whole piece fits now, but it can then no longer be resumed.
            if (room_() < used_ || !write_(segment_, used_)) return pause_or_fail(true);
            mid_leaf_written_ = true;
            used_ = 0;
        }
        segment_[used_++] = c;
        return true;
    }

    bool commit() {
        if (used_ == 0) {
            mid_leaf_written_ = false;
            return true;
        }
        if (room_() < used_) return pause_or_fail(mid_leaf_written_);
        if (!write_(segment_, used_)) return fail();
        used_ = 0;
        mid_leaf_written_ = false;
        return true;
    }

    bool pause_or_fail(bool cannot_resume) {
        result_ = cannot_resume ? RestPassResult::Failed : RestPassResult::Paused;
        return false;
    }

    bool fail() {
        result_ = RestPassResult::Failed;
        return false;
    }

    RestResumeJob& job_;
    Write write_;
    Room room_;
    JsonLeafLexer lexer_;
    char segment_[SegmentCapacity];
    size_t used_ = 0;
    uint32_t leaf_index_ = 0;
    uint32_t crc_ = 0;
    bool mid_leaf_written_ = false;
    RestPassResult result_ = RestPassResult::Complete;
};

} // namespace signalk_mini
