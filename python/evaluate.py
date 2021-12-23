# Run experiments based on given "parameters"
import os
import sys
import copy
import threading
from datetime import datetime


# ----- Functions ----- #

def prod(array):
    l = len(array)
    p = 1
    for i in range(0,l):
        p = p*array[i]
    return p

def getCombinationsTable(parameters):

    # Sizes of lists of parameters
    sizes = []
    for i in range(0, len(parameters)):
        sizes.append( len(parameters[i]) )

    # Generate all the necessary empty lists
    n_lists = len(sizes)
    lists = []
    for i in range(0, n_lists):
        lists.append(list())

    # Fill in all the lists
    n_combs = prod(sizes)
    #print("Number of combinations: " + str(n_combs))
    sizes_rem = copy.copy(sizes)


    for i in range(0, n_lists):
    
        #Constants
        if len(sizes_rem) > 0:
            sizes_rem.pop(0)
            n_repets = prod(sizes_rem)
            #print("Number of local repetitions: " +str(n_repets))
        else :
            n_repets = 1

        n_repets_global = int( n_combs/(sizes[i]*n_repets) )
        #print("Number of global repetitions: "+ str(n_repets_global))

        # Fill in
    
        for j in range(0, n_repets_global):
            for k in parameters[i]:
                for z in range(0, n_repets): 
                    lists[i].append(k)
                
    return lists

def thread_task(lock, table, name, bag_location, bag_file_path):
    global index
    l_table = len(table[0])

    if index <= l_table-1 :
        lock.acquire()
        index += 1
        local_index = copy.copy(index)
        lock.release()

        # Generate directory for results
        now = datetime.now()
        dt_string   = now.strftime("%Y-%m-%d_%H:%M:%S")
        dt_string_2 = now.strftime("%Y-%m-%d_%H")
        eval_name = 'Test_number_'+ str(local_index) + '_' + dt_string
        output_eval_path = bag_location + '/CoralRadarEval/' + eval_name
        os.system('mkdir -p ' + output_eval_path)

        # Initialize ROS node instance
        range_error = table[0][local_index-1]
        method = table[1][local_index-1]
        param_str = name + ' launched test ' + str(local_index) + ' with ' + method + ' and ' + str(range_error) 
        print(param_str)
        launch_str = 'rosrun alignment_checker evaluate_scans --input-file-path ' + bag_file_path + ' --output-dir ' + output_eval_path + ' --eval-name ' + eval_name + ' --sequence ' + sequence + ' --method ' + method + ' --range-error ' + str(range_error) + ' --scan-type cfear --rosbag-offset 200 --frame-delay 0.0 --visualization false __name:=Test_' + str(local_index)
        os.system(launch_str)
        
        # Append line to file with these parameters
        f = open(bag_location + '/CoralRadarEval/' + 'testsLogInfo_'+ dt_string_2 +'.txt', "a")
        f.writelines([param_str+'\n'])
        f.close

    else : 
        return


# ----- Main routine ----- #

if __name__ == '__main__' :

    # Get parameters
    filename = 'parameters.py'
    exec(compile(open(filename, 'rb').read(), filename, 'exec'))
    tableParams = getCombinationsTable(parameters)

    # Initialization
    global index
    index = 0
    l_table = len(tableParams[0])
    #n_threads = int( input('Number of threads to be used: ') )
    n_threads = int(float(sys.argv[1]))
    threads = [None]*n_threads

    # Get rosbag base dir and others
    bag_location = os.getenv('BAG_LOCATION')
    bag_base_dir = bag_location + '/oxford-eval-sequences'
    sequence = '2019-01-10-12-32-52-radar-oxford-10k'
    bag_file_path = bag_base_dir + '/' + sequence + '/radar/' + sequence + '.bag'

    # Mutex
    lock = threading.Lock()

    # Executing tasks
    while index <= (l_table-1):
        
        #Creating threads
        for i in range(0, n_threads):
            name = 'Thread_' + str(i+1)
            threads[i] = threading.Thread(target=thread_task, args=(lock,tableParams,name,bag_location,bag_file_path))

        #Starting threads
        for i in range(0, n_threads):
            threads[i].start()

        #Waiting for threads
        for i in range(0, n_threads):
            threads[i].join()
    
