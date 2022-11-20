docker build . -t odm/report_creater
docker run -ti -v /datasets:/datasets odm/report_creater --project-path /datasets project --auto-boundary --dsm
