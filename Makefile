.DEFAULT_GOAL := all

TOP := $(abspath ..)
include $(TOP)/mk/common.mk

PROJECT := sdhc
OUT     := $(BUILD_ROOT)/$(PROJECT)
TARGET  := $(OUT)/$(PROJECT).a

SRCS := \
	src/sdhc.c

OBJS := $(addprefix $(OUT)/,$(notdir $(SRCS:.c=.o)))
DEPS := $(OBJS:.o=.d)

SYS_INCLUDES := \
	-isystem$(TOP)/freertos/src/inc

USER_INCLUDES := \
	-I$(TOP)/inc \
	-I$(TOP)/sys/src \
	-I$(TOP)/ucdrv/src

WARNINGS := \
	-Wall \
	-Wextra \
	-Wstrict-prototypes \
	-Wmissing-prototypes \
	-Wmissing-declarations \
	-Wshadow \
	-Wpointer-arith \
	-Wbad-function-cast \
	-Wcast-align \
	-Wcast-qual \
	-Wjump-misses-init \
	-Wno-unused-parameter \
	-Wundef

OPT_FLAGS := -O2
CC1_FLAGS := $(CW_CC1_PRE_FLAGS) $(SYS_INCLUDES) $(USER_INCLUDES) $(CW_COMMON_DEFS)
CC1_POST_FLAGS := $(CW_CC1_BASE_FLAGS) $(WARNINGS) $(OPT_FLAGS) $(CW_CODEGEN_FLAGS)
AS_FLAGS := --traditional-format $(USER_INCLUDES) $(CPU_AS_FLAGS)

.PHONY: all clean
all: check-toolchain $(TARGET)

$(TARGET): $(OBJS)
	@echo "  AR      $@"
	@rm -f "$@"
	$(AR) -rcs "$@" $(OBJS)

$(OUT)/%.o: src/%.c | $(OUT)
	@echo "  CC      $<"
	$(CC1) $(CC1_FLAGS) -MD "$(OUT)/$*.d" -MQ "$@" $(CC1_POST_FLAGS) "$<" -o "$(OUT)/$*.asm"
	$(AS) $(AS_FLAGS) "$(OUT)/$*.asm" -o "$@"
	@rm -f "$(OUT)/$*.asm"

$(OUT):
	@mkdir -p "$@"

clean:
	@rm -rf "$(OUT)"

-include $(DEPS)
