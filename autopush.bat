@echo off

:: 获取当前的ISO 8601格式日期和时间（格式：YYYY-MM-DDTHH:MM:SS）
for /f "tokens=2 delims==" %%a in ('wmic os get localdatetime /value') do set datetime=%%a
set "datetimestamp=%datetime:~0,4%-%datetime:~4,2%-%datetime:~6,2%T%datetime:~8,2%:%datetime:~10,2%:%datetime:~12,2%"

:: 进行git操作
git add .
git commit -m "update %datetimestamp%"
git push --force

echo Git操作完成: 已经添加、提交并推送了代码。
