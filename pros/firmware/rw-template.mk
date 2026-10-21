# From the RW-Template PROS template. RW's library uses your devices and settings by name
# (src/rw-config.cpp), so it links with your code instead of going in the cold package
EXCLUDE_COLD_LIBRARIES += $(FWDIR)/RW-Template.a
