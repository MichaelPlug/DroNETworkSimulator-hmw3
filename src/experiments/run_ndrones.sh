#-----------------------------------------------------------#
#           _  _ ___  ___  ___  _  _ ___ ___                #
#          | \| |   \| _ \/ _ \| \| | __/ __|               #
#          | .` | |) |   / (_) | .` | _|\__ \               #
#          |_|\_|___/|_|_\\___/|_|\_|___|___/               #
#                                                           #
#-----------------------------------------------------------#

#test baselines
for nd in "2" "5" "10" "15" "20" "25" "30" "35" "40";
do
    for alg in "MGEO" "AI";
    # if you experienced too much time to run experiments, remove "GEO" and "RND"
    do
        echo "run: ${alg} - ndrones ${nd} "
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 3 -e_s 5 -alg ${alg} &
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 10 -e_s 20 -alg ${alg} &
        python3 -m src.experiments.experiment_ndrones -nd ${nd} -i_s 20 -e_s 30 -alg ${alg} &
    done;
done;
wait
python3 -m src.experiments.json_and_plot -nd 2 -nd 5 -nd 10 -nd 15 -nd 20 -nd 25 -nd 30 -nd 35 -nd 40 -i_s 20 -e_s 30 -exp_suffix MGEO -exp_suffix AI
