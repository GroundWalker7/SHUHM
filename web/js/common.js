console.log(1)
setEchart();
setBtn();


function setEchart() {
    var echart1 = echarts.init(document.getElementById("echart1"));
    echart1.setOption({
        title: {
            textStyle: {
                fontSize: 30
            },
            text: '一周血糖浓度曲线'
        },
        tooltip: {
            trigger: 'axis'
        },
        visualMap: {
            top: 10,
            right: 10,
            pieces: [{
                gt: 0,
                lte: 3.9,
                color: '#FFB90F'
            }, {
                gt: 3.9,
                lte: 6.1,
                color: '#228B22'
            }, {
                gt: 6.1,
                color: 'red'
            }]
        },
        xAxis: {
            axisLabel: {
                fontSize: 20
            },
            type: 'category',
            data: ['6-14', '6-15', '6-16', '6-17', '6-18', '6-19', '6-20', '6-21', '6-22', '6-23', '6-24', '6-25', '6-26', '6-27', '6-28', '6-29', '6-30', '7-01', '7-02', '7-03', '7-04', '7-05', '7-06', '7-07', '7-08', '7-09', '7-10', '7-11', '7-12', '7-13']
        },
        yAxis: {
            axisLabel: {
                fontSize: 20
            },
            type: 'value',
            min: 2
        },
        series: [{
            smooth: 0.4,
            name: '血糖浓度',
            // symbol: 'circle',
            symbolSize: 8,
            data: [4.3, 4.9, 5.4, 3.6, 6.1, 5.0, 5.7, 7.2, 4.1, 5.5, 4.2, 4.5, 6.2, 3.3, 6.5, 5.8, 7.2, 3.3, 6.7, 5.9, 6.4, 5.7, 4.7, 3.0, 6.3, 6.2, 4.1, 4.1, 3.2, 3.6],
            type: 'line',
            color: 'green',
            itemStyle: {
                normal: {
                    // color: '#6cb041',
                    lineStyle: {
                        width: 9//设置线条粗细
                    }
                }
            },
            markLine: {
                silent: true,
                data: [{
                    yAxis: 3.9
                }, {
                    yAxis: 6.1
                }],
                itemStyle: {
                    normal: {
                        lineStyle: { 
                            width:2,
                            color: "black"
                        },
                        // fontSize: 20
                    },
                }
            },
            markPoint: {
                symbolSize: 80,
                data: [
                    { type: 'max', name: '最大值' },
                    { type: 'min', name: '最小值' }
                ],
                itemStyle: {
                    normal: {
                        label: {
                            show: true,
                            fontSize: 25
                        },
                        color: "#B22222"
                    }
                },

            }
        }],

    });

    var echart2 = echarts.init(document.getElementById("echart2"));
    echart2.setOption({
        title: {
            textStyle: {
                fontSize: 30
            },
            text: '一周尿酸浓度曲线'
        },
        tooltip: {
            trigger: 'axis'
        },
        visualMap: {
            top: 10,
            right: 10,
            pieces: [{
                gt: 0,
                lte: 89,
                color: '#FFB90F'
            }, {
                gt: 89,
                lte: 416,
                color: '#228B22'
            }, {
                gt: 416,
                color: 'red'
            }]
        },
        xAxis: {
            axisLabel: {
                fontSize: 20
            },
            type: 'category',
            data: ['6-14', '6-15', '6-16', '6-17', '6-18', '6-19', '6-20', '6-21', '6-22', '6-23', '6-24', '6-25', '6-26', '6-27', '6-28', '6-29', '6-30', '7-01', '7-02', '7-03', '7-04', '7-05', '7-06', '7-07', '7-08', '7-09', '7-10', '7-11', '7-12', '7-13']
        },
        yAxis: {
            axisLabel: {
                fontSize: 20
            },
            type: 'value',
            min: 50
        },
        series: [{
            smooth: 0.4,
            name: '尿酸浓度',
            // symbol: 'circle',
            symbolSize: 8,
            data: [278, 160, 308, 404, 254, 341, 254, 130, 63, 295, 481, 395, 85, 461, 231, 383, 395, 103, 413, 231, 160, 228, 346, 252, 168, 482, 300, 387, 111, 317],
            type: 'line',
            color: 'green',
            itemStyle: {
                normal: {
                    // color: '#6cb041',
                    lineStyle: {
                        width: 9//设置线条粗细
                    }
                }
            },
            markLine: {
                silent: true,
                data: [{
                    yAxis: 89
                }, {
                    yAxis: 416
                }],
                itemStyle: {
                    normal: {
                        lineStyle: { 
                            width:2,
                            color: "black"
                        }
                    },
                    
                }
            },
            markPoint: {
                symbolSize: 80,
                data: [
                    { type: 'max', name: '最大值' },
                    { type: 'min', name: '最小值' }
                ],
                itemStyle: {
                    normal: {
                        label: {
                            show: true,
                            fontSize: 25
                        },
                        color: "#B22222"
                    }
                },

            }
        }],

    });
}

function setBtn() {
    var btn1 = document.getElementById("btn1");
    var data1 = document.getElementById("data1")
    btn1.onclick = function() {
        pywebview.api.getData().then(function(r) {
            data1.innerHTML = r + "    mmol/L";
        });
    }
    // btn1.onclick = function() {
        
    //     window.data1.innerHTML = "111";

    // }
}