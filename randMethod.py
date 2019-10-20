from MCMPinstance import MCMPInstance
import random
from MCMPstcDE.evaluator import STCEvaluator
import  sys

if __name__ == '__main__':
    print('begin the random method')
    ins = MCMPInstance()

    confDir = 'D:\\pycode\\MCMP_encode\\benchmark\\'
    if len(sys.argv) == 4:
        fileName = sys.argv[1]
        # maxSampleTimes = int(sys.argv[2])
        ins.loadCfg(confDir + fileName + '.dat')
        random.seed(1)
        runTimes  = int(sys.argv[2])
    else:
        print('pycharm run')
        fileName = 'r2_r40_c20_p0.9_s1000_Outdoor_Cfg'
        ins.loadCfg(confDir + fileName + '.dat')
        runTimes = 60000
        random.seed(1)

    stc_eval = STCEvaluator(ins)
    NDIM = stc_eval._stcGraph.number_of_nodes()
    fileCfg = './/resData//' + fileName + '//rand//ndim_' + str(NDIM) + 'res_rand.dat'
    f_con = open(fileCfg, 'w')
    makespanLst = []
    for sampleTime in range(runTimes):
        pop = []
        for i in range(NDIM):
            pop.append((random.random(), random.random(), random.random()))

        x = [[] for x in range(stc_eval._robNum)]
        robID = 0
        for ind in pop:
            x[robID].append(ind)
            if robID == (stc_eval._robNum - 1):
                robID = 0
            else:
                robID += 1

        valid, makespan = stc_eval.evaluate(x)
        if valid:
            makespanLst.append(makespan)
            f_con.write(str(makespan) + '\n')
            f_con.flush()
            print(min(makespanLst))
    minMakespan = min(makespanLst)
    f_con.write('runTimes = ' + str(runTimes) + '\n')
    f_con.write('minMakespan = ' + str(minMakespan) + '\n')
    f_con.close()
    print('end random method')
