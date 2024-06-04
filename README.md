Markov Localization for Discrete Map

This code implements Markov Localization for a robot navigating on a discrete map .

Key Features:

    Unknown initial robot position (belief-based approach)
    Fixed robot orientation
    Movement model:
        60% chance - moves 3 cells
        15% chance - moves 2 or 4 cells (each direction)
        5% chance - moves 1 or 5 cells (each direction)
    Lidar simulation (from HW3, Q3):
        40% probability - detects obstacle correctly
        7.5% probability - detects obstacle in adjacent cell

Functionality:

    Robot moves for 20 cycles (or until position belief reaches 70% certainty)
    Probability distribution of robot's location is updated in each iteration
    Colormap visualizes the robot's position belief

Potential Use Cases:

    Robotics research
    Robot self-localization
    Monte Carlo localization simulations
<img width="419" alt="correct" src="https://github.com/AlpMercan/Markov-Localization/assets/112685013/3a128b77-8592-4b31-9da4-d92502407022">
<img width="413" alt="correct2" src="https://github.com/AlpMercan/Markov-Localization/assets/112685013/77e387f7-8aac-4ecb-953e-d6f4c340caa2">
<img width="416" alt="%70Percent" src="https://github.com/AlpMercan/Markov-Localization/assets/112685013/cb79b8b8-d890-40ff-b57d-2f6532554d78">
<img width="412" alt="Yanlış" src="https://github.com/AlpMercan/Markov-Localization/assets/112685013/ae594d4f-8ddc-46cc-8b86-dafa9d2f4c52">
