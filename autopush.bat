@echo off

:: 设置日期和时间格式
set "BackupTime=%date:~-4,4%%date:~-10,2%%date:~-7,2%_%time:~0,2%%time:~3,2%%time:~6,2%"

:: 进行git操作
git add .
git commit -m "update %BackupTime%"
git push --force

echo Git操作完成: 已经添加、提交并推送了代码。
