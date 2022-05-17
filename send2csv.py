import csv

def send2csv(filename, array):
    with open(filename, 'w', newline = '') as file:
        writer = csv.writer(file, delimiter = ',')
        #writer.writerow(array)
        for a in array:
            writer.writerow(a)
    return
