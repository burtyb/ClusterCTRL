DIRS = clusteraplus6-1.0 clusterphat-1.0 clusterda-1.0 clustertriple-1.0 clustersingle-1.0
.PHONY: all clean

all:
	for dir in $(DIRS); do \
		$(MAKE) -C $$dir; \
	done

clean: 
	for dir in $(DIRS); do \
		$(MAKE) -C $$dir clean; \
	done

