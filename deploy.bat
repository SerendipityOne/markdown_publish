@echo off
chcp 65001 > nul
echo 正在激活conda mkdocs环境...
call conda activate mkdocs
if %errorlevel% neq 0 (
    echo 环境激活失败
    pause
    exit /b
)

echo 正在构建文档...
mkdocs build
if %errorlevel% neq 0 (
    echo 构建失败
    pause
    exit /b
)

echo 正在部署到GitHub Pages...
mkdocs gh-deploy
if %errorlevel% neq 0 (
    echo 部署失败
    pause
    exit /b
)

echo 所有操作已完成!
pause
