#include "alu.h"

#include "common/polyfills/mulh64.h"

namespace machine {

RegisterValue alu_combined_operate(
    AluCombinedOp op,
    AluComponent component,
    bool w_operation,
    bool modified,
    RegisterValue a,
    RegisterValue b) {
    switch (component) {
    case AluComponent::ALU:
        return (w_operation) ? alu32_operate(op.alu_op, modified, a, b)
                             : alu64_operate(op.alu_op, modified, a, b);
    case AluComponent::MUL:
        return (w_operation) ? mul32_operate(op.mul_op, a, b) : mul64_operate(op.mul_op, a, b);
    case AluComponent::PASS:
        return a;
    default: qDebug("ERROR, unknown alu component: %hhx", uint8_t(component)); return 0;
    }
}

/**
 * Shift operations are limited to shift by 31 bits.
 * Other bits of the operand may be used as flags and need to be masked out
 * before any ALU operation is performed.
 */
constexpr uint64_t SHIFT_MASK32 = 0b011111; // == 31
constexpr uint64_t SHIFT_MASK64 = 0b111111; // == 63

int64_t alu64_operate(AluOp op, bool modified, RegisterValue a, RegisterValue b) {
    uint64_t _a = a.as_u64();
    uint64_t _b = b.as_u64();

    switch (op) {
    case AluOp::ADD: return _a + ((modified) ? -_b : _b);
    case AluOp::SLL: return _a << (_b & SHIFT_MASK64);
    case AluOp::SLT: return a.as_i64() < b.as_i64();
    case AluOp::SLTU: return _a < _b;
    case AluOp::XOR:
        return _a ^ _b;
        // Most compilers should calculate SRA correctly, but it is UB.
    case AluOp::SR:
        return (modified) ? (a.as_i64() >> (_b & SHIFT_MASK64)) : (_a >> (_b & SHIFT_MASK64));
    case AluOp::OR: return _a | _b;
    case AluOp::AND:
        return ((modified) ? ~_a : _a) & _b; // Modified: clear bits of b using mask
                                             // in a
    default: qDebug("ERROR, unknown alu operation: %hhx", uint8_t(op)); return 0;
    }
}

int32_t alu32_operate(AluOp op, bool modified, RegisterValue a, RegisterValue b) {
    uint32_t _a = a.as_u32();
    uint32_t _b = b.as_u32();

    switch (op) {
    case AluOp::ADD: return _a + ((modified) ? -_b : _b);
    case AluOp::SLL: return _a << (_b & SHIFT_MASK32);
    case AluOp::SLT: return a.as_i32() < b.as_i32();
    case AluOp::SLTU: return _a < _b;
    case AluOp::XOR:
        return _a ^ _b;
        // Most compilers should calculate SRA correctly, but it is UB.
    case AluOp::SR:
        return (modified) ? (a.as_i32() >> (_b & SHIFT_MASK32)) : (_a >> (_b & SHIFT_MASK32));
    case AluOp::OR: return _a | _b;
    case AluOp::AND:
        return ((modified) ? ~_a : _a) & _b; // Modified: clear bits of b using mask in a
    default: qDebug("ERROR, unknown alu operation: %hhx", uint8_t(op)); return 0;
    }
}

int64_t mul64_operate(MulOp op, RegisterValue a, RegisterValue b) {
    switch (op) {
    case MulOp::MUL: return a.as_u64() * b.as_u64();
    case MulOp::MULH: return mulh64(a.as_i64(), b.as_i64());
    case MulOp::MULHSU: return mulhsu64(a.as_i64(), b.as_u64());
    case MulOp::MULHU: return mulhu64(a.as_u64(), b.as_u64());
    case MulOp::DIV:
        if (b.as_i64() == 0) {
            return -1; // Division by zero is defined.
        } else if (a.as_i64() == INT64_MIN && b.as_i64() == -1) {
            return INT64_MIN; // Overflow.
        } else {
            return a.as_i64() / b.as_i64();
        }
    case MulOp::DIVU:
        return (b.as_u64() == 0) ? UINT64_MAX // Division by zero is defined.
                                 : a.as_u64() / b.as_u64();
    case MulOp::REM:
        if (b.as_i64() == 0) {
            return a.as_i64(); // Division by zero is defined.
        } else if (a.as_i64() == INT64_MIN && b.as_i64() == -1) {
            return 0; // Overflow.
        } else {
            return a.as_i64() % b.as_i64();
        }
    case MulOp::REMU:
        return (b.as_u64() == 0) ? a.as_u64() // Division by zero reminder
                                              // is defined.
                                 : a.as_u64() % b.as_u64();
    default: qDebug("ERROR, unknown multiplication operation: %hhx", uint8_t(op)); return 0;
    }
}

int32_t mul32_operate(MulOp op, RegisterValue a, RegisterValue b) {
    switch (op) {
    case MulOp::MUL: return a.as_u32() * b.as_u32();
    case MulOp::MULH: return ((uint64_t)a.as_i32() * (uint64_t)b.as_i32()) >> 32;
    case MulOp::MULHSU: return ((uint64_t)a.as_i32() * (uint64_t)b.as_u32()) >> 32;
    case MulOp::MULHU: return ((uint64_t)a.as_u32() * (uint64_t)b.as_u32()) >> 32;
    case MulOp::DIV:
        if (b.as_i32() == 0) {
            return -1; // Division by zero is defined.
        } else if (a.as_i32() == INT32_MIN && b.as_i32() == -1) {
            return INT32_MIN; // Overflow.
        } else {
            return a.as_i32() / b.as_i32();
        }
    case MulOp::DIVU:
        return (b.as_u32() == 0) ? UINT32_MAX // Division by zero is defined.
                                 : a.as_u32() / b.as_u32();
    case MulOp::REM:
        if (b.as_i32() == 0) {
            return a.as_i32(); // Division by zero is defined.
        } else if (a.as_i32() == INT32_MIN && b.as_i32() == -1) {
            return 0; // Overflow.
        } else {
            return a.as_i32() % b.as_i32();
        }
    case MulOp::REMU:
        return (b.as_u32() == 0) ? a.as_u32() // Division by zero reminder
                                              // is defined.
                                 : a.as_u32() % b.as_u32();
    default: qDebug("ERROR, unknown multiplication operation: %hhx", uint8_t(op)); return 0;
    }
}

// 设置向量长度
void vsetvl(RegisterValue &vl, RegisterValue &type, RegisterValue rs1, RegisterValue rs2) {
    // 设置向量长度 (vl) 和类型 (type)
    vl = rs1;  // 设置向量长度为 rs1
    type = rs2;  // 设置向量元素类型为 rs2（8/16/32）
}

// 向量加法
void vadd_vv(RegisterValue &vd, const RegisterValue &vs1, const RegisterValue &vs2, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        vd[i] = vs1[i] + vs2[i];  // 对应元素相加
    }
}

// 向量与标量加法
void vadd_vx(RegisterValue &vd, const RegisterValue &vs2, RegisterValue rs1, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        vd[i] = vs2[i] + rs1;  // 每个向量元素加上标量 rs1
    }
}
// 向量与立即数加法
void vadd_vi(RegisterValue &vd, const RegisterValue &vs2, RegisterValue imm, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        vd[i] = vs2[i] + imm;  // 每个向量元素加上立即数 imm
    }
}

// 向量点积
void vmul_vv(RegisterValue &vd, const RegisterValue &vs1, const RegisterValue &vs2, size_t length) {
    RegisterValue dot_product = 0;
    for (size_t i = 0; i < length; ++i) {
        dot_product += vs1[i] * vs2[i];  // 逐元素相乘
    }
    vd = dot_product;  // 将点积结果存储到 vd
}

// 加载向量元素
void vlw_v(RegisterValue &vd, RegisterValue rs1, RegisterValue vl, RegisterValue type, Memory &memory) {
    size_t length = vl.as_u64();
    size_t element_size = (type.as_u64() == 8) ? 1 : (type.as_u64() == 16) ? 2 : 4;
    
    for (size_t i = 0; i < length; ++i) {
        vd[i] = memory.load(rs1 + i * element_size);  // 从内存加载元素
    }
}

// 存储向量元素
void vsw_v(const RegisterValue &vs3, RegisterValue rs1, RegisterValue vl, RegisterValue type, Memory &memory) {
    size_t length = vl.as_u64();
    size_t element_size = (type.as_u64() == 8) ? 1 : (type.as_u64() == 16) ? 2 : 4;
    
    for (size_t i = 0; i < length; ++i) {
        memory.store(rs1 + i * element_size, vs3[i]);  // 将元素存储到内存
    }
}

RegisterValue execute_rvv_instruction(const Instruction &inst, const RegisterValue &alu_fst, const RegisterValue &alu_sec) {
    RegisterValue result;

    switch (inst.get_opcode()) {
        case RVV_OPCODE_VSETVL:
            // vsetvl rd, rs1, rs2: 设置向量长度寄存器和向量类型
            result = vsetvl(inst);
            break;
        
        case RVV_OPCODE_VADD_VV:
            // vadd.vv vd, vs2, vs1: 向量相加
            result = vadd_vv(inst, alu_fst, alu_sec);
            break;
        
        case RVV_OPCODE_VADD_VX:
            // vadd.vx vd, vs2, rs1: 向量与标量相加
            result = vadd_vx(inst, alu_fst, alu_sec);
            break;
        
        case RVV_OPCODE_VADD_VI:
            // vadd.vi vd, vs2, imm: 向量与标量值相加
            result = vadd_vi(inst, alu_fst, alu_sec);
            break;
        
        case RVV_OPCODE_VMUL_VV:
            // vmul.vv vd, vs2, vs1: 向量点积
            result = vmul_vv(inst, alu_fst, alu_sec);
            break;
        
        case RVV_OPCODE_VLW_V:
            // vlw.v vd, (rs1): 加载向量
            result = vlw_v(inst, alu_fst);
            break;
        
        case RVV_OPCODE_VSW_V:
            // vsw.v vs3, (rs1): 存储向量
            result = vsw_v(inst, alu_fst);
            break;
        
        default:
            // 无效的 RVV 操作
            throw std::runtime_error("Unsupported RVV operation.");
    }

    return result;
}

void process_rvv_memory(const Instruction &inst, const RegisterValue &alu_fst, const RegisterValue &alu_sec) {
    switch (inst.get_opcode()) {
        case RVV_OPCODE_VLW_V:
            // 处理向量加载操作
            // 例如，根据 vl 寄存器的值确定加载的长度
            load_vector(inst, alu_fst);
            break;

        case RVV_OPCODE_VSW_V:
            // 处理向量存储操作
            // 例如，根据 vl 寄存器的值确定存储的长度
            store_vector(inst, alu_fst);
            break;

        default:
            // 不需要内存访问的 RVV 指令
            break;
    }
}

void write_rvv_back(const Instruction &inst, const RegisterValue &result) {
    switch (inst.get_opcode()) {
        case RVV_OPCODE_VSETVL:
            // 对 vsetvl 指令，不需要写回数据
            break;

        case RVV_OPCODE_VADD_VV:
        case RVV_OPCODE_VADD_VX:
        case RVV_OPCODE_VADD_VI:
        case RVV_OPCODE_VMUL_VV:
            // 对于加法和乘法指令，写回结果到寄存器
            write_vector_result(inst, result);
            break;

        case RVV_OPCODE_VLW_V:
            // 对于加载指令，已在内存中存储结果
            break;

        case RVV_OPCODE_VSW_V:
            // 对于存储指令，不需要写回寄存器
            break;

        default:
            // 不需要写回数据的 RVV 操作
            break;
    }
}


} // namespace machine
