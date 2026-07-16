import os.path
import numpy as np
import matplotlib.pyplot as plt
import util
import random

def learn_distributions(file_lists_by_category):
    """
    Estimate the parameters p_d, and q_d from the training set
    
    Input
    -----
    file_lists_by_category: A two-element list. The first element is a list of 
    spam files, and the second element is a list of ham files.

    Output
    ------
    probabilities_by_category: A two-element tuple. The first element is a dict 
    whose keys are words, and whose values are the smoothed estimates of p_d;
    the second element is a dict whose keys are words, and whose values are the 
    smoothed estimates of q_d 
    """
    ### TODO: Write your code here
    spam_files = file_lists_by_category[0]
    ham_files = file_lists_by_category[1]

    spam_word_counts = util.get_word_freq(spam_files)
    ham_word_counts = util.get_word_freq(ham_files)

    #For counting vocabulary, we use set to ensure each word appear once in set
    vocabulary = set(spam_word_counts.keys()) | set(ham_word_counts.keys())

    D = len(vocabulary)
    alpha = 1  # Laplace smoothing

    # Total number of words in each class
    total_spam_words = sum(spam_word_counts.values())
    total_ham_words = sum(ham_word_counts.values())

    p_d = {}
    q_d = {}

    #Laplace smoothing
    for word in vocabulary:
        p_d[word] = (spam_word_counts[word] + alpha) / \
                       (total_spam_words + alpha * D)

        q_d[word] = (ham_word_counts[word] + alpha) / \
                       (total_ham_words + alpha * D)

        probabilities_by_category = (p_d, q_d)
    
    return probabilities_by_category

def classify_new_email(filename,probabilities_by_category,prior_by_category, threshold):
    """
    Use Naive Bayes classification to classify the email in the given file.

    Inputs
    ------
    filename: name of the file to be classified
    probabilities_by_category: output of function learn_distributions
    prior_by_category: A two-element list as [\pi, 1-\pi], where \pi is the 
    parameter in the prior class distribution

    Output
    ------
    classify_result: A two-element tuple. The first element is a string whose value
    is either 'spam' or 'ham' depending on the classification result, and the 
    second element is a two-element list as [log p(y=1|x), log p(y=0|x)], 
    representing the log posterior probabilities
    """
    ### TODO: Write your code here
    p_d, q_d = probabilities_by_category
    pi = prior_by_category[0]
    prior_ham = prior_by_category[1]
    #list of all words in file named filename
    words = util.get_words_in_file(filename)
    MAP_spam = np.log(pi)
    MAP_ham  = np.log(prior_ham)
    
    for word in words:
        if word in p_d:
            MAP_spam += np.log(p_d[word])
            MAP_ham  += np.log(q_d[word])
        else:
            continue #ignore other words not in vacabulary
    


    if MAP_spam > MAP_ham + threshold:
        classify_result = ("spam", [MAP_spam, MAP_ham])
    else:
        classify_result = ("ham", [MAP_spam, MAP_ham])
    
    return classify_result

def select_files(directory, fraction=0.7):
    """
    This function builds a customized dataset for each group

    """
    all_files = [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.txt')]
    random.shuffle(all_files)
    num_files = int(len(all_files) * fraction)
    return all_files[:num_files]


if __name__ == '__main__':
    
    ############################CHANGE YOUR STUDENT ID###############################
    student1_number = 1010390989  # Replace with the actual student number
    student2_number = 1008686888  # Replace with the actual student number
    random.seed((student1_number+student2_number)/1000)
    
    # folder for training and testing 
    spam_folder = "data/spam"
    ham_folder = "data/ham"
    test_folder = "data/testing"

    # generate the file lists for training
    file_lists = []
    file_lists = [select_files(folder) for folder in (spam_folder, ham_folder)]
        
    # Learn the distributions    
    probabilities_by_category = learn_distributions(file_lists)
    
    # prior class distribution
    priors_by_category = [0.5, 0.5]
    
    # Store the classification results
    performance_measures = np.zeros([2,2])
    # explanation of performance_measures:
    # columns and rows are indexed by 0 = 'spam' and 1 = 'ham'
    # rows correspond to true label, columns correspond to guessed label
    # to be more clear, performance_measures = [[p1 p2]
    #                                           [p3 p4]]
    # p1 = Number of emails whose true label is 'spam' and classified as 'spam' 
    # p2 = Number of emails whose true label is 'spam' and classified as 'ham' 
    # p3 = Number of emails whose true label is 'ham' and classified as 'spam' 
    # p4 = Number of emails whose true label is 'ham' and classified as 'ham' 

    # Classify emails from testing set and measure the performance
    thresholds = np.linspace(-20, 20, 20)

    type1_errors_list = []
    type2_errors_list = []
    
    for t in thresholds:
    
        performance_measures = np.zeros([2,2])
    
        for filename in util.get_files_in_folder(test_folder):
    
            label, _ = classify_new_email(filename,
                                           probabilities_by_category,
                                           priors_by_category,
                                           threshold=t)
    
            base = os.path.basename(filename)
            true_index = ('ham' in base)
            guessed_index = (label == 'ham')
    
            performance_measures[int(true_index), int(guessed_index)] += 1
    
        # Extract errors
        type1_errors = performance_measures[0,1]  # spam -> ham
        type2_errors = performance_measures[1,0]  # ham -> spam
    
        type1_errors_list.append(type1_errors)
        type2_errors_list.append(type2_errors)
    
    # Plot once
    plt.figure()
    plt.plot(type1_errors_list, type2_errors_list, marker='o')
    plt.xlabel('Type 1 errors')
    plt.ylabel('Type 2 errors')
    plt.title('Type 1 vs Type 2 Error Tradeoff')
    plt.grid(True)
    plt.show()


    template="You correctly classified %d out of %d spam emails, and %d out of %d ham emails."
    # Correct counts are on the diagonal
    correct = np.diag(performance_measures)
    # totals are obtained by summing across guessed labels
    totals = np.sum(performance_measures, 1)
    print(template % (correct[0],totals[0],correct[1],totals[1]))
    
    
    ### TODO: Write your code here to modify the decision rule such that
    ### Type 1 and Type 2 errors can be traded off, plot the trade-off curve
   

 