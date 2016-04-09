var express = require('express');
var router = express.Router();
var fft = require('fft-js').fft,
    fftUtil = require('fft-js').util;

var data = {};

/* GET home page. */
router.get('/:student', function(req, res) {
  res.render('index', { title: 'Express' });
});

router.get('/:student/signal', function(req, res) {
  res.status(200).json(data[req.params.student]);
});

/* POST home page. */
router.post('/:student/signal', function(req, res) {
  if(req.body.data != undefined){
    if(data[req.params.student] == undefined){
      data[req.params.student] = {};
    }
    data[req.params.student]["signal"] = req.body.data;
    if(req.body.max != undefined){
        data[req.params.student]["max"] = req.body.max;
    }




    var signal = [];
    for(var i=0; i<req.body.data.length;i++){
      signal.push(data[req.params.student]["signal"][i][1]);
    }
    var phasors = fft(signal);

    var sampleRate = data[req.params.student]["sampleRate"] | 44000;
    console.log(sampleRate);

    var frequencies = fftUtil.fftFreq(phasors, sampleRate), // Sample rate and coef is just used for length, and frequency step
        magnitudes = fftUtil.fftMag(phasors);

    var both = frequencies.map(function (f, ix) {
      return [f,magnitudes[ix]];
    });
    data[req.params.student]["fft"] = both;

    res.send(200);
  }else{
    res.send("Wrong data format!");
  }

});

router.get('/:student/fft', function(req, res) {
  res.status(200).json(data[req.params.student]);
});

/* POST home page. */
router.post('/:student/fft', function(req, res) {
  console.log(req.body);
  if(req.body.data != undefined){
    if(data[req.params.student] == undefined){
      data[req.params.student] = {};
    }
    data[req.params.student]["fft"] = req.body.data;
    if(req.body.max != undefined){
        data[req.params.student]["max"] = req.body.max;
    }
    res.send(200);
  }else{
    res.send("Wrong data format!");
  }

});


module.exports = router;
