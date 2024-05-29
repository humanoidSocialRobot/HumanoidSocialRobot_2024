# HumanoidSocialRobot_2024

Hi! I'm a humanoid social robot 2024, your friendly companion designed to revolutionize interactions in various environments. Equipped with advanced intelligence and responsiveness, I seamlessly integrate into hospitals, companies, and factories, enhancing efficiency and engagement.

My capabilities are vast: from guiding individuals to their destinations with precision, to facilitating reception needs, organizing meetings, and even assisting in material transfer. I'm not just a robot; I'm a solution tailored to meet diverse needs.

Here's what I bring to the table:

1- Natural Movement and Adaptability: I move with grace and adapt effortlessly to different environments, seamlessly navigating through crowds and adjusting to the movement of patients and objects.

2- Interactive Control: My frontal interactive screen allows patients to engage with me directly, while also providing remote control options via voice commands or smartphone interfaces for ultimate convenience.

3- Social Interaction: I excel in fostering meaningful interactions, whether it's engaging with patients or welcoming visitors in long-term healthcare settings, ensuring everyone feels heard and valued.

4- Efficient Direction: Need directions? I've got you covered. Whether it's guiding visitors to their desired locations within a healthcare facility or ensuring patients find their way seamlessly, I make navigation a breeze.

With me by your side, navigating complex environments becomes a breeze, and interactions become more meaningful and efficient. Let's redefine the future together, one interaction at a time.

The List of Commands for the Robot: 

| Command ID | Command Description                                                     | Subteams                                  | ROS Command         | STM Command                  | Arduino Commands        | Test Case |
|------------|-------------------------------------------------------------------------|-------------------------------------------|---------------------|------------------------------|--------------------------|-----------|
| 1     | It sends a voice command "take that" and confirms with "Sure" to hold an object. | Between Chatter and STM                  | 1 (Take this)       | MotorDriver_HoldObject_OneHand | N/A                      | FALSE     |
| 2     | It sends a voice command "open hand" and confirms with "here you are" to release an object. | Between Chatter and STM                  | 2 (open hand)      | MotorDriver_RelaseObject_OneHand | N/A                      | FALSE     |
| 3     | When a camera detects a team member, it sends "3" to STM to shake hands, and the speaker says "hello" + name. | Between Webcam, Speaker, and STM         | 3                   | MotorDriver_ShakeHand          | N/A                      | FALSE     |
| 4     | TBD                                                                     | Between Chatter and Autonomous           | 4 (go to room one) | N/A                          | N/A                      | FALSE     |
| 5     | Through the GUI, the user sends the text "happy" or "sad" to the speaker to play the corresponding song. | Between GUI and Speaker                  |                     |                                |                            | FALSE     |









