uint16 height           # image height, that is, number of rows.
uint16 width            # image width, that is, number of columns.

# all arrays will be equal in length
uint64[] timestamp      # timestamp of each event.
uint64[] x              # x position (column) of each event, from left.
uint64[] y              # y position (row) of each event, from top.
uint64[] polarity         # polarity of each event. 1=positive, 0=negative.