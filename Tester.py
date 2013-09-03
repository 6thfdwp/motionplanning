from motionplanning.ASVPlanner import ASVPlanner

# testcases = ['3ASV-t1.txt', '3ASV-variable-x4.txt', '3ASV-fixed-x4.txt', '7ASV-x4.txt'] 
# each test case could be configured with a set of parameters.
# each pair represents the number of samples (e.g., 150) and radius when connecting close samples
# tuningPara = [ [(150,0.1), (150,0.2), (200,0.1) (200,0.2)], \
#               [(150,0.1), (150,0.2), (200,0.1) (200,0.2)], \
#               [(150,0.1), (150,0.2), (200,0.1) (200,0.2)], \
#               [(150,0.1), (150,0.2), (200,0.1) (200,0.2)] ]

testcases = ['7ASV-3obs.txt'] 
tuningPara = [ [(400,0.16)] ]
for i, case in enumerate(testcases):
    paras = tuningPara[i]
    for para in paras: # try different samples and radius
        sampleNum, radius = para
        ASVPlanner(case, sampleNum, radius)
