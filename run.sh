DIR=$(pwd)/result/
if [ -d "$DIR" ];
then
    echo "dir already exists"
else
	git clone https://github.com/OpenDroneMap/odm_data_aukerman.git result
fi
docker build . -t odm/report_creater --no-cache
docker run -ti -v $(pwd)/result/:/datasets/project/ odm/report_creater --project-path /datasets project --auto-boundary --dsm
