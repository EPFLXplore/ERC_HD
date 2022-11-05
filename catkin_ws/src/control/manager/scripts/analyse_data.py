import sys
import matplotlib.pyplot as plt


def numeric(s):
    return s in ("0", "1", "2", "3", "4", "5", "6", "7", "8", "9")


def main():
    file = "../../../../res22"
    pos_results = {}
    neg_results = {}
    with open(file, "r") as f:
        f.readline()
        for line in f:
            if not(line[0] == "\t"): continue
            j = 1
            while numeric(line[j]):
                j += 1
            rpm = int(line[1:j])
            i = line.find(":")+2
            offset = float(line[i:].strip())
            if offset < 0:
                neg_results.setdefault(rpm, []).append(-offset)
            else:
                pos_results.setdefault(rpm, []).append(offset)
    
    pos = []
    pos_val = []
    neg = []
    neg_val = []
    for k in pos_results:
        for x in pos_results[k]:
            pos.append(k)
            pos_val.append(x)
        #pos.append(k)
        #pos_val.append(sum(pos_results[k])/len(pos_results[k]))
    for k in neg_results:
        for x in pos_results[k]:
            neg.append(k)
            neg_val.append(x)
        #neg.append(k)
        #neg_val.append(sum(neg_results[k])/len(neg_results[k]))
    
    plt.plot(pos, pos_val, ".", label='pos', color="r")
    plt.plot(neg, neg_val, ".", label='neg', color="b")
    plt.show()
    print("done")


if __name__ == "__main__":
    main()
