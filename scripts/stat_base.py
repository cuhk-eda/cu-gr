import re

class RegexPattern:
    def __init__(self, keys, stringPattern, active=True, activeCondition=None):
        self.keys = keys
        self.pattern = re.compile(stringPattern)    # for speed up
        self.active = active
        self.initActive = active
        self.activeCondition = activeCondition
    def ProcessLine(self, result, line):
        if not self.active:
            if self.activeCondition != None and self.activeCondition in line:
                self.active = True # activate
            return False
        m = self.pattern.match(line)
        if m:
            for i, k in enumerate(self.keys):
                assert(k not in result)
                result[k] = m.group(i+1)
            self.active = False
            return True # inserted
        else:
            return False
    def Reset(self):
        self.active = self.initActive

class RegexPatternSet:
    def __init__(self):
        self.patterns = []
        self.cnt = 0
    def Add(self, *args, **kwargs):
        self.patterns.append(RegexPattern(*args, **kwargs))
    def ProcessLine(self, result, line):
        for pattern in self.patterns:
            if pattern.ProcessLine(result, line):
                self.cnt += 1
                return self.cnt
    def Reset(self):
        for pattern in self.patterns:
            pattern.Reset()
        self.cnt = 0

def GetVal(item, result):
    return result[item] if item in result else "N/A"

def PrintResultTable(items, results):
    # calculate widths
    widths = [len(item) for item in items]
    for result in results:
        for i, item in enumerate(items):
            val = result[item] if item in result else "N/A"
            if len(val) > widths[i]:
                widths[i] = len(val)
    # print items
    for i in range(len(items) - 1):
        print("{:{}}".format(items[i], widths[i]), end=" | ")
    print("{:{}}".format(items[-1], widths[-1]))
    for i in range(len(items) - 1):
        if i == 0:
            print(':' + '-' * (widths[i] - 1) + ':|-', end='')
        else:
            print('-' * widths[i] + ':|-', end='')

    print('-' * (widths[-1]-1) + ':')
    # print values
    for result in results:
        for i in range(len(items) - 1):
            print("{:{}}".format(GetVal(items[i], result), widths[i]), end=" | ")
        print("{:{}}".format(GetVal(items[-1], result), widths[-1]))
