import csv

def find_path(eid, input_file, output_file="path.csv"):
    path = [eid]
    with open(input_file, mode='r') as infile:
        reader = csv.DictReader(infile)
        rows = {row['eid']: row for row in reader}

        current_eid = eid
        while current_eid != '0':
            if current_eid not in rows:
                print(f"Eid {current_eid} not found in the input file.")
                return
            if rows[current_eid]['sid'] == '0':
                break
            path.append(rows[current_eid]['sid'])
            current_eid = rows[current_eid]['sid']

    with open(output_file, mode='w', newline='') as outfile:
        writer = csv.DictWriter(outfile, fieldnames=reader.fieldnames)
        writer.writeheader()
        for eid in reversed(path):
            if eid in rows:
                writer.writerow(rows[eid])

find_path('540', 'logs/tree_edges.csv', 'logs/path.csv')