
ABUSE_H = ../../include/linux/abuse.h
ABUSE_HEADER_FILE = $(notdir $(ABUSE_H))
ABUSECTL = abusectl

CFLAGS += -Wall -Wextra -O2

$(ABUSECTL): $(ABUSE_HEADER_FILE)

$(ABUSE_HEADER_FILE):
	@ln -s $(ABUSE_H)

clean:
	@$(RM) $(ABUSECTL)
	@$(RM) $(ABUSE_HEADER_FILE)

.PHONY: clean
