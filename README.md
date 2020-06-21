# Ballbot Matlab Control Simulation
Matlab simulation of the CMU Ballbot

## Install 
1. Move ot the direction `ballbot_ctrl_sim`
2. Run the script `install_ballbot_sim.m`. 

## Dependencies
| Name | Description|
| -------- | -------- |
| [ballbot_viz]()   | Matlab ballbot animation and visualization tools |

## Models Available 

## Running Examples
Example simulation are stored in the direction `ballbot_ctrl_sim/examples`. You can run any script inside it and it should simulate ballbot.

If you want to simulate ballbot with your own controller I suggest creating a very similar script to the one in the examples directory. 

## Changing Model Parameters
Ballbot model parameteres are stored in  `models\get_ballbot2D_model_params.m`

Feel free to change the values in this script to match your system

## Making Your Own Controllers
To make your own conntroller you need to create a new script under the `controllers` directory the follows a similar structure

## Models

### Ballbot 2D with 2DoF Arms
Currently there is no working example of this model balancing and performing arm motion. You can run the test script to try and get it to balance
in `ballbot_ctrl_sim/test`.

Run the test script
```
test_ballbot2D_w2DOFArm_dyns.m
```
