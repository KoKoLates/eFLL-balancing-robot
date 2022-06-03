source_file = kalman_filter.c kalman_filter.h
object_file = kalman_filter

$(object_file):$(source_file)
	gcc -g $(source_file) -o $@

.PHONY:

clean:
	-rm -f *.o $(object_file) tags

runs:
	./$(object_file) > object.txt