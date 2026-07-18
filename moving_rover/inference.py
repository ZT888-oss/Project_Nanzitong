import numpy as np
import graphics
import rover

def forward_backward(all_possible_hidden_states,
                     all_possible_observed_states,
                     prior_distribution,
                     transition_model,
                     observation_model,
                     observations):
    """
    Inputs
    ------
    all_possible_hidden_states: a list of possible hidden states
    all_possible_observed_states: a list of possible observed states
    prior_distribution: a distribution over states

    transition_model: a function that takes a hidden state and returns a
        Distribution for the next state
    observation_model: a function that takes a hidden state and returns a
        Distribution for the observation from that hidden state
    observations: a list of observations, one per hidden state
        (a missing observation is encoded as None)

    Output
    ------
    A list of marginal distributions at each time step; each distribution
    should be encoded as a Distribution (see the Distribution class in
    rover.py), and the i-th Distribution should correspond to time
    step i
    """

    num_time_steps = len(observations)
    forward_messages = [None] * num_time_steps
    forward_messages[0] = prior_distribution
    backward_messages = [None] * num_time_steps
    marginals = [None] * num_time_steps 
    
    # TODO: Compute the forward messages
    forward_messages[0] = rover.Distribution()

    for z in all_possible_hidden_states:
        prob = prior_distribution[z]
        if observations[0] is not None:
            prob *= observation_model(z)[observations[0]]
        forward_messages[0][z] = prob

    forward_messages[0].renormalize()

    for i in range(1, num_time_steps):
        forward_messages[i] = rover.Distribution()

        for z in all_possible_hidden_states:
            total = 0.0
            for zp in all_possible_hidden_states:
                total += forward_messages[i-1][zp] * transition_model(zp)[z]

            if observations[i] is not None:
                total *= observation_model(z)[observations[i]]

            forward_messages[i][z] = total

        forward_messages[i].renormalize()

                   
    # TODO: Compute the backward messages
    backward_messages[num_time_steps - 1] = rover.Distribution()
    for z in all_possible_hidden_states:
        backward_messages[num_time_steps - 1][z] = 1.0
    backward_messages[num_time_steps - 1].renormalize()

    for i in reversed(range(num_time_steps - 1)):
        backward_messages[i] = rover.Distribution()

        for z in all_possible_hidden_states:
            total = 0.0
            for zn in all_possible_hidden_states:
                prob = transition_model(z)[zn]

                if observations[i+1] is not None:
                    prob *= observation_model(zn)[observations[i+1]]

                prob *= backward_messages[i+1][zn]
                total += prob

            backward_messages[i][z] = total

        backward_messages[i].renormalize()

    # TODO: Compute the marginals 
    for i in range(num_time_steps):
        marginals[i] = rover.Distribution()

        for z in all_possible_hidden_states:
            marginals[i][z] = forward_messages[i][z] * backward_messages[i][z]

        marginals[i].renormalize()

    return marginals
def Viterbi(all_possible_hidden_states,
            all_possible_observed_states,
            prior_distribution,
            transition_model,
            observation_model,
            observations):

    import numpy as np

    num_time_steps = len(observations)

    delta = [{} for _ in range(num_time_steps)]
    psi = [{} for _ in range(num_time_steps)]

    # ---------- INIT ----------
    for z in all_possible_hidden_states:
        p = prior_distribution[z]

        if observations[0] is not None:
            p *= observation_model(z)[observations[0]]

        delta[0][z] = np.log(p + 1e-12)
        psi[0][z] = None

    # ---------- RECURSION ----------
    for i in range(1, num_time_steps):
        for z in all_possible_hidden_states:

            best_prev = None
            best_val = float('-inf')

            obs_prob = 1.0
            if observations[i] is not None:
                obs_prob = observation_model(z)[observations[i]]

            for zp in all_possible_hidden_states:

                trans_prob = transition_model(zp)[z]

                val = delta[i-1][zp] + np.log(trans_prob + 1e-12) + np.log(obs_prob + 1e-12)

                if val > best_val:
                    best_val = val
                    best_prev = zp

            delta[i][z] = best_val
            psi[i][z] = best_prev

    # ---------- BACKTRACK ----------
    estimated_states = [None] * num_time_steps

    last_state = max(delta[-1], key=delta[-1].get)
    estimated_states[-1] = last_state

    for i in reversed(range(num_time_steps - 1)):
        estimated_states[i] = psi[i+1][estimated_states[i+1]]
        
        
        
        
    # ---- VITERBI ERROR ----
    viterbi_correct = sum(
        1 for i in range(100)
        if estimated_states[i] == hidden_states[i]
    )
    
    Pe_viterbi = 1 - viterbi_correct / 100
    
    
    # ---- FB MAP STATES ----
    fb_estimated_states = [
        max(marginals[i], key=marginals[i].get)
        for i in range(100)
    ]
    
    fb_correct = sum(
        1 for i in range(100)
        if fb_estimated_states[i] == hidden_states[i]
    )
    
    Pe_fb = 1 - fb_correct / 100
    
    
    print("Viterbi error Pe_tilde =", Pe_viterbi)
    print("Forward-backward error Pe_hat =", Pe_fb)
    
    fb_states = [
        max(marginals[i], key=marginals[i].get)
        for i in range(len(marginals))
    ]
    
    for i in range(len(fb_states) - 1):
        curr = fb_states[i]
        nxt = fb_states[i + 1]
    
        if transition_model(curr)[nxt] == 0:
            print("INVALID TRANSITION FOUND")
            print("i =", i)
            print("z_i =", curr)
            print("z_{i+1} =", nxt)
            break 
    return estimated_states




if __name__ == '__main__':
   
    enable_graphics = True
    
    missing_observations = True
    if missing_observations:
        filename = 'test_missing.txt'
    else:
        filename = 'test.txt'
            
    # load data    
    hidden_states, observations = rover.load_data(filename)
    num_time_steps = len(hidden_states)

    all_possible_hidden_states   = rover.get_all_hidden_states()
    all_possible_observed_states = rover.get_all_observed_states()
    prior_distribution           = rover.initial_distribution()
    
    print('Running forward-backward...')
    marginals = forward_backward(all_possible_hidden_states,
                                 all_possible_observed_states,
                                 prior_distribution,
                                 rover.transition_model,
                                 rover.observation_model,
                                 observations)
    print('\n')


   
    timestep = num_time_steps - 1
    #timestep = 30
    print("Most likely parts of marginal at time %d:" % (timestep))
    print(sorted(marginals[timestep].items(), key=lambda x: x[1], reverse=True)[:10])
    print('\n')

    print('Running Viterbi...')
    estimated_states = Viterbi(all_possible_hidden_states,
                               all_possible_observed_states,
                               prior_distribution,
                               rover.transition_model,
                               rover.observation_model,
                               observations)
    print('\n')
    
    print("Last 10 hidden states in the MAP estimate:")
    for time_step in range(num_time_steps - 10, num_time_steps):
        print(estimated_states[time_step])
  
    # if you haven't complete the algorithms, to use the visualization tool
    # let estimated_states = [None]*num_time_steps, marginals = [None]*num_time_steps
    # estimated_states = [None]*num_time_steps
    # marginals = [None]*num_time_steps
    if enable_graphics:
        app = graphics.playback_positions(hidden_states,
                                          observations,
                                          estimated_states,
                                          marginals)
        app.mainloop()
        
