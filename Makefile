# Target binary name
OUT=CPU

# Source files
SOURCES=template_lab2.v tb_lab2.v lib_lab2.v

# Rule for the default target
all: $(OUT)

# Compile rule
$(OUT): $(SOURCES)
	iverilog -o $(OUT) $(SOURCES)

# Clean rule to remove generated files
clean:
	rm -f $(OUT)

.PHONY: all clean