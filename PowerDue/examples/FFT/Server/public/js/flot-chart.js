var Script = function () {

//  tracking chart

    var plot, plot1;
    $(function () {
        var data = [[0,1],[1,1]];

        plot = $.plot($("#chart-2"),
            [ { data: data, label: "sin(x) = -0.00"} ], {
                series: {
                    lines: { show: true }
                },
                crosshair: { mode: "x" },
                grid: { hoverable: true, autoHighlight: false },
                //yaxis: { min: Math.min.apply(Math, sin), max: Math.max.apply(Math, sin) }
                yaxis: { min: 0, max: 20 }
            });
        var legends = $("#chart-2 .legendLabel");

        legends.each(function () {
            // fix the widths so they don't jump around
            $(this).css('width', $(this).width());
        });

        var updateLegendTimeout = null;
        var latestPosition = null;

        function updateLegend() {
            updateLegendTimeout = null;

            var pos = latestPosition;

            var axes = plot.getAxes();
            if (pos.x < axes.xaxis.min || pos.x > axes.xaxis.max ||
                pos.y < axes.yaxis.min || pos.y > axes.yaxis.max)
                return;

            var i, j, dataset = plot.getData();
            for (i = 0; i < dataset.length; ++i) {
                var series = dataset[i];

                // find the nearest points, x-wise
                for (j = 0; j < series.data.length; ++j)
                    if (series.data[j][0] > pos.x)
                        break;

                // now interpolate
                var y, p1 = series.data[j - 1], p2 = series.data[j];
                if (p1 == null)
                    y = p2[1];
                else if (p2 == null)
                    y = p1[1];
                else
                    y = p1[1] + (p2[1] - p1[1]) * (pos.x - p1[0]) / (p2[0] - p1[0]);

                legends.eq(i).text(series.label.replace(/=.*/, "= " + y.toFixed(2)));
            }
        }

        $("#chart-2").bind("plothover",  function (event, pos, item) {
            latestPosition = pos;
            if (!updateLegendTimeout)
                updateLegendTimeout = setTimeout(updateLegend, 50);
        });

        function updateGraph(){
          $.ajax({
              url: window.location.pathname + "/signal"
          }).done(function(data) {
            var axes = plot.getAxes();
            axes.yaxis.options.max = data.signal[0][1];
            for(var i=0;i<data.signal.length;i++){

              if(data.signal[i][1] > axes.yaxis.options.max){

                axes.yaxis.options.max = data.signal[i][1];
              }
            }
            plot.setData([data.signal]);
            plot.setupGrid();
            plot.draw();
            setTimeout(updateGraph, 500);
          });
        }
        updateGraph();

    });


//    live chart
$(function () {
    var data = [[0,1],[1,1]];

    plot1 = $.plot($("#chart-1"),
        [ { data: data, label: "sin(x) = -0.00"} ], {
            series: {
                lines: { show: true }
            },
            crosshair: { mode: "x" },
            grid: { hoverable: true, autoHighlight: false },
            //yaxis: { min: Math.min.apply(Math, sin), max: Math.max.apply(Math, sin) }
            yaxis: { min: 0, max: 20 }
        });
    var legends = $("#chart-1 .legendLabel");

    legends.each(function () {
        // fix the widths so they don't jump around
        $(this).css('width', $(this).width());
    });

    var updateLegendTimeout = null;
    var latestPosition = null;

    function updateLegend() {
        updateLegendTimeout = null;

        var pos = latestPosition;

        var axes = plot.getAxes();
        if (pos.x < axes.xaxis.min || pos.x > axes.xaxis.max ||
            pos.y < axes.yaxis.min || pos.y > axes.yaxis.max)
            return;

        var i, j, dataset = plot.getData();
        for (i = 0; i < dataset.length; ++i) {
            var series = dataset[i];

            // find the nearest points, x-wise
            for (j = 0; j < series.data.length; ++j)
                if (series.data[j][0] > pos.x)
                    break;

            // now interpolate
            var y, p1 = series.data[j - 1], p2 = series.data[j];
            if (p1 == null)
                y = p2[1];
            else if (p2 == null)
                y = p1[1];
            else
                y = p1[1] + (p2[1] - p1[1]) * (pos.x - p1[0]) / (p2[0] - p1[0]);

            legends.eq(i).text(series.label.replace(/=.*/, "= " + y.toFixed(2)));
        }
    }

    $("#chart-1").bind("plothover",  function (event, pos, item) {
        latestPosition = pos;
        if (!updateLegendTimeout)
            updateLegendTimeout = setTimeout(updateLegend, 50);
    });

    function updateGraph1(){
      $.ajax({
          url: window.location.pathname + "/fft"
      }).done(function(data) {
        var axes = plot1.getAxes();
        axes.yaxis.options.max = data.fft[0][1];
        for(var i=0;i<data.fft.length;i++){
          if(data.fft[i][1] > axes.yaxis.options.max){
            axes.yaxis.options.max = data.fft[i][1];
          }
        }

        plot1.setData([data.fft]);
        plot1.setupGrid();
        plot1.draw();
        setTimeout(updateGraph1, 500);
      });
    }
    updateGraph1();

});




}();
