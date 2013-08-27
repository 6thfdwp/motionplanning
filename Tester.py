from motionplanning.ASVPlanner import ASVPlanner

testcases = ['3ASV-t1.txt', '3ASV-variable-x4.txt', '3ASV-fixed-x4.txt', '7ASV-x4.txt'] 
tuningPara = [ [(150,0.1), (150,0.2), (200,0.1) (200,0.2)], [(150,0.1), (150,0.2), (200,0.1) (200,0.2)], [(150,0.1), (150,0.2), (200,0.1) (200,0.2)], [(150,0.1), (150,0.2), (200,0.1) (200,0.2)] ]

for i, case in enumerate(testcases):
    paras = tuningPara[i]
    for para in paras: # may try different samples and radius
        sampleNum, radius = para
        ASVPlanner(case, sampleNum, radius)
