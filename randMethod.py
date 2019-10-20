from MCMPinstance import MCMPInstance
import random
from MCMPstcDE.evaluator import STCEvaluator
import  sys

if __name__ == '__main__':
    print('begin the random method')
    ins = MCMPInstance()
    confDir = 'D:\\pycode\\MCMP_encode\\benchmark\\'
    if len(sys.argv) == 3:
        fileName = sys.argv[1]
        maxSampleTimes = int(sys.argv[2])
        ins.loadCfg(confDir + fileName + '.dat')
        # random.seed(seed)
        runTimes  = int(sys.argv[3])
    else:
        print('pycharm run')
        fileName = 'r2_r40_c20_p0.9_s1000_Outdoor_Cfg'
        ins.loadCfg(confDir + fileName + '.dat')
        runTimes = 2000

    stc_eval = STCEvaluator(ins)
    fileCfg = './/resData//' + fileName + '//rand//res_rand.dat'
    f_con = open(fileCfg, 'w')
    makespanLst = []
    for sampleTime in range(runTimes):
        pop = []
        for i in range(300):
            pop.append((random.random(), random.random(), random.random()))

        valid, makespan = stc_eval.evaluate(pop)
        if valid:
            makespanLst.append(makespan)
            f_con.write(str(makespan) + '\n')
            f_con.flush()
    minMakespan = min(makespanLst)
    f_con.write('runTimes = ' + str(runTimes) + '\n')
    f_con.write('minMakespan = ' + str(minMakespan) + '\n')
    f_con.close()
    print('end random method')
