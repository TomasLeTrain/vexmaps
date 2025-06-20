#include "pros/apix.h"
#include <arm_neon.h>
#include <array>
#include <cassert>
#include <cstring>
#include <deque>
#include <list>
#include <memory>
#include <vector>


namespace vexmaps {
namespace logger {

// message:
// we asssume both sides know the data types - magics
//
// each [] is a byte, each () is a varint
//
// [message magic 1][message magic 2](size in bytes)[child1][child2]...
// [children magic1][children magic2](size in bytes)[child1][child2]...
// ...

typedef union {
    float_t a;
    uint8_t b[4];
} float_char_t;

typedef union {
    float16_t a;
    uint8_t b[2];
} float16_char_t;

// order for all is kept the same since it might matter for types like floats
inline void writeToList(std::list<char>& list, float_char_t data) {
    list.push_back(data.b[0]);
    list.push_back(data.b[1]);
    list.push_back(data.b[2]);
    list.push_back(data.b[3]);
}

inline void writeToList(std::list<char>& list, char* data, int len) {
    for(int i = 0;i < len;i++)
        list.push_back(data[i]);
}

inline void writeToList(std::list<char>& list, float16_char_t data) {
    list.push_back(data.b[0]);
    list.push_back(data.b[1]);
}

// TODO: switch to using varints for integers
inline void writeToList(std::list<char>& list, int32_t data) {
    list.push_back(data & 0xff);
    list.push_back((data >> 8) & 0xff);
    list.push_back((data >> 16) & 0xff);
    list.push_back((data >> 24) & 0xff);
}

class BaseMessageLogger {
  public:
    BaseMessageLogger() {}

    // this should be a general magic for the type of message (category, data
    // type, etc.)
    virtual char getMagic1() = 0;
    // this should be specific to the field
    virtual char getMagic2() = 0;

    virtual std::list<char> LogData() = 0;
    virtual std::vector<BaseMessageLogger*> getChildren() = 0;

    virtual ~BaseMessageLogger() = default;
};

class CategoryLogger : public BaseMessageLogger {
    static constexpr char basicDataTypeMagic = 0x70;

  public:
    char getMagic1() override {
        return basicDataTypeMagic;
    }

    // since its a structure this should never get called
    std::list<char> LogData() override {
        return {};
    }
};

class BaseTypeLogger : public BaseMessageLogger {
    static constexpr char basicDataTypeMagic = 0x71;

  public:
    char getMagic1() override {
        return basicDataTypeMagic;
    }
    // since its only data this should always return empty
    std::vector<BaseMessageLogger*> getChildren() override {
        return {};
    };
};

class IntLogger : public BaseTypeLogger {
  private:
    int data;
    static constexpr char intMagic = 0x11;

  public:
    IntLogger() {}
    IntLogger(int data) : data(data) {}

    void setData(int data){
        this->data = data;
    }

    char getMagic2() override {
        return intMagic;
    }

    std::list<char> LogData() override {
        std::list<char> res;
        // for now we dont add the first magic since this a basic type
        res.push_back(getMagic2());
        writeToList(res, data);
        return res;
    }

    ~IntLogger() override = default;
};

class FloatLogger : public BaseTypeLogger {
  private:
    float data;
    static constexpr char floatMagic = 0x12;

  public:
    FloatLogger() {}
    FloatLogger(float data) : data(data) {}

    void setData(float data){
        this->data = data;
    }

    char getMagic2() override {
        return floatMagic;
    }

    std::list<char> LogData() override {
        std::list<char> res;
        // for now we dont add the first magic since this a basic type
        res.push_back(getMagic2());
        writeToList(res, (float_char_t)data);
        return res;
    }

    ~FloatLogger() override = default;
};

class PoseLogger : public BaseTypeLogger {
  private:
    float x;
    float y;
    float z;
    static constexpr char poseMagic = 0x13;

  public:
    PoseLogger() {}
    PoseLogger(float x, float y, float z)
        : x(x),
          y(y),
          z(z) {}

    void setData(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }

    char getMagic2() override {
        return poseMagic;
    }

    std::list<char> LogData() override {
        std::list<char> res;
        // for now we dont add the first magic since this a basic type
        res.push_back(getMagic2());
        writeToList(res, (float_char_t)x);
        writeToList(res, (float_char_t)y);
        writeToList(res, (float_char_t)z);
        return res;
    }

    ~PoseLogger() override = default;
};

template<size_t N>
class ParticlesLogger : public BaseTypeLogger {
  private:
    float16_t x[N];
    float16_t y[N];
    float16_t weights[N];

    static constexpr char particleLoggerMagic = 0x41;

  public:
    char getMagic2() override {
        return particleLoggerMagic;
    }

    void addParticles(float* x,
                      float* y,
                      float* weights,
                      const size_t len,
                      const size_t offset = 0) {
        // end index in our array
        const size_t n = len + offset;
        assert((n <= N) && "given more elements than length of logger");

        const size_t min_n = std::min(N, n);

        // offset should not matter
        const size_t remaining_particles = (min_n - (min_n % 4));

        for (size_t i = offset; i < remaining_particles; i += 4) {
            // load in values
            float32x4_t vx = vld1q_f32(&x[i]);
            float32x4_t vy = vld1q_f32(&y[i]);
            float32x4_t vweights = vld1q_f32(&weights[i]);

            // convert to float16
            float16x4_t v16x = vcvt_f16_f32(vx);
            float16x4_t v16y = vcvt_f16_f32(vy);
            float16x4_t v16weights = vcvt_f16_f32(vweights);

            // load to internal arrays
            vst1_f16(&(this->x[i]), v16x);
            vst1_f16(&(this->y[i]), v16y);
            vst1_f16(&(this->weights[i]), v16weights);
        }
        for (size_t i = remaining_particles; i < min_n; i++) {
            this->x[i] = x[i];
            this->y[i] = y[i];
            this->weights[i] = weights[i];
        }
    }

    void setParticle(const size_t i, float x, float y, float weight) {
        this->x[i] = x;
        this->y[i] = y;
        this->weights[i] = weight;
    }

    std::list<char> LogData() {
        // TODO: might be able to compress further if we remove the last two
        // bits of the mantissa from x/y

        std::list<char> result;

        // since this is a normal datatype we have to handle the length and
        // magic logic ourselves
        // This should probably be changed eventually
        for (int i = 0; i < N; i++) {
            writeToList(result, (float16_char_t)x[i]);
            writeToList(result, (float16_char_t)y[i]);
            writeToList(result, (float16_char_t)weights[i]);
        }
        // TODO: switch to varint's for this
        result.push_front(result.size());

        result.push_front(getMagic2());
        result.push_front(getMagic1());
    }

    ~ParticlesLogger() override = default;
};

class GenerationInfoLogger : public CategoryLogger {
  private:
    static constexpr char generationInfoMagic = 0x40;

    std::vector<BaseMessageLogger*> children {};

  public:
    FloatLogger timestamp;

    char getMagic2() override {
        return generationInfoMagic;
    }

    std::vector<BaseMessageLogger*> getChildren() override {
        return children;
    };

    ~GenerationInfoLogger() override = default;
};

/**
 * @brief Holds all the information being printed by the PF
 */
template<size_t N>
class PFLogger : public CategoryLogger {
  private:
    std::vector<BaseMessageLogger*> children { &generation_info, &particles };

    static constexpr char PFMagic = 0xaf;

  public:
    GenerationInfoLogger generation_info;
    ParticlesLogger<N> particles;

    std::vector<BaseMessageLogger*> getChildren() override {
        return children;
    }
};

// traversing list in dfs order
// assumes list is getting copied
inline std::list<char> buildData(BaseMessageLogger* current_message) {
    auto children = current_message->getChildren();

    if (children.empty()) {
        // not a structure, just data
        return current_message->LogData();
    }

    std::list<char> result;

    for (auto curr : children) {
        std::list<char> curr_list = buildData(curr);
        result.splice(result.end(), curr_list);
    }
    size_t current_len = result.size();

    // put current_len in front of the list - keeps order of bytes the same
    // TODO: switch to varint's
    result.push_front((current_len >> 24) & 0xff);
    result.push_front((current_len >> 16) & 0xff);
    result.push_front((current_len >> 8) & 0xff);
    result.push_front((current_len) & 0xff);

    // append magics
    result.push_front(current_message->getMagic2());
    result.push_front(current_message->getMagic1());

    return result;
}

inline void sendData(BaseMessageLogger* message) {
    std::list<char> data = buildData(message);
    std::vector<char> buffer(data.begin(), data.end());

    FILE* sout = fopen("sout", "w");
    fwrite(buffer.data(), sizeof(char), buffer.size(), sout);
    fclose(sout);
}

} // namespace logger
} // namespace vexmaps
