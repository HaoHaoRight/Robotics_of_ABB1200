@echo off

:: 获取当前日期和时间
for /F "tokens=1-6 delims=/: " %%a in ('echo %date% %time%') do (
    set dd=%%a
    set mm=%%b
    set yyyy=%%c
    set hh=%%d
    set min=%%e
)
set "datetimestamp=%dd%/%mm%/%yyyy% %hh%:%min%"

:: 进行git操作
git add .
git commit -m "update %datetimestamp%"
git push origin main --force

echo Git操作完成: 已经添加、提交并推送了代码。

