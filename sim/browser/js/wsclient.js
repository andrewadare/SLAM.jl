$( function() {

  // Initiate a WebSocket connection
  var ws = new WebSocket( 'ws://' + window.location.host );

  // Set up the scene SVG element ASAP to avoid reflowing
  var width = 800;
  var height = 800;
  var xscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ 0, width ]);
  var yscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ height, 0 ]);
  var ryscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ 0, height ]);
  var scene = d3.select( '.scene' )
      .attr( 'width', width )
      .attr( 'height', height );

  // Streaming data for vehicle track (ideal and inferred)
  var simTrack = [];
  var slamTrack = [];
  var simTrackSvg = scene.append( 'g' )
      .attr( 'class', 'simtrack');
  var slamTrackSvg = scene.append( 'g' )
      .attr( 'class', 'slamtrack');
  var lidarLines = scene.append( 'g' )
      .attr( 'class', 'lidar-lines' );
  var vehicleEllipse = scene.append( 'g' )
      .attr( 'class', 'vehicle-ellipse' )
      .append( 'ellipse' );

  function drawWaypoints( data ) {
    scene.selectAll( '.waypoints' )
      .data( data )
      .enter()
      .append( 'circle' )
      .attr( 'r', 0 ) // px
      .transition()
      .duration( 750 )
      .attr( 'r', 4 ) // px
      .attr( 'cx', function( d ) { return xscale( d.x ); })
      .attr( 'cy', function( d ) { return yscale( d.y ); })
      .attr( 'class', 'waypoints' );
  }

  function drawLandmarks( data ) {

    var l = 10; // Edge length of square

    var landmarks = scene.selectAll( 'g .landmarks' )
      .data( data )
      .enter()
      .append( 'g' )
      .attr( 'class', 'landmarks' )
      .attr( 'transform', function( d ) {
        return 'translate(' + (xscale( d.x ) - l/2) + ',' + (yscale( d.y ) - l/2) + ')';
      });

    landmarks.append( 'rect' )
      .attr( 'width', 0 )
      .attr( 'height', 0 )
      .transition()
      .duration( 750 )
      .attr( 'width', l )
      .attr( 'height', l )
      .attr( 'rx', l/10)
      .attr( 'ry', l/10);
  }


  function drawSimTrack( data ) {

    simTrack.push( {
      'x': xscale( data.ideal.x ),
      'y': yscale( data.ideal.y )
    });

    slamTrack.push( {
      'x': xscale( data.slam.x ),
      'y': yscale( data.slam.y )
    });

    var n = simTrack.length;

    if ( n > 1 ) {
      simTrackSvg
        .append( 'line' )
        .attr( 'x1', simTrack[ n - 2 ].x )
        .attr( 'y1', simTrack[ n - 2 ].y )
        .attr( 'x2', simTrack[ n - 1 ].x )
        .attr( 'y2', simTrack[ n - 1 ].y );

      slamTrackSvg
        .append( 'line' )
        .attr( 'x1', slamTrack[ n - 2 ].x )
        .attr( 'y1', slamTrack[ n - 2 ].y )
        .attr( 'x2', slamTrack[ n - 1 ].x )
        .attr( 'y2', slamTrack[ n - 1 ].y );
    }
  }

  function drawLidar( data ) {
    var i = -1;
    while ( ++i < data.length ) {
      scene.selectAll( '.lidar-lines' )
        .append( 'line' )
        .attr( 'x1', xscale( data[ i ].x1 ) )
        .attr( 'y1', yscale( data[ i ].y1 ) )
        .attr( 'x2', xscale( data[ i ].x2 ) )
        .attr( 'y2', yscale( data[ i ].y2 ) )
        .transition()
        .duration( 500 )
        .style( 'opacity', 0 )
        .each( 'end', function() {
          d3.select( this )
            .remove();
        });
    }
  }

  function drawVehicleEllipse( data ) {
    var d = data[0];
    var nSigma = 2;
    vehicleEllipse
      .attr( 'rx',  xscale( nSigma*d.rx ) )
      .attr( 'ry', ryscale( nSigma*d.ry ) )
      .attr( 'transform',
             'translate(' + xscale( d.cx ) + ',' + yscale( d.cy ) + ') ' +
             'rotate(' + -d.phi*180/Math.PI + ')' );
  }

  function drawFeatures( data ) {
    var nSigma = 2;

    // Since y is up in the simulation, but upside down in SVG land, we
    // negate phi to get the correct CCW rotation.
    function transform( d ) {
      return 'translate(' + xscale( d.cx ) + ',' + yscale( d.cy ) + ') ' +
             'rotate(' + -d.phi*180/Math.PI + ')';
    }
    function rx( d ) {
      return xscale( nSigma*d.rx );
    }
    function ry( d ) {
      return ryscale( nSigma*d.ry );
    }

    // Display feature symbols
    scene.selectAll( '.feature' )
      .data( data )
      .attr( 'transform', transform )
      .enter().append( 'path' )
      .attr( 'class', 'feature' )
      .attr( 'd', d3.svg.symbol().type( 'circle' ) )
      .attr( 'transform', transform );

    // Display feature uncertainty ellipses
    scene.selectAll( '.feature-ellipse' )
      .data( data )
      .attr( 'rx', rx )
      .attr( 'ry', ry )
      .attr( 'transform', transform )
      .enter().append( 'ellipse' )
      .attr( 'class', 'feature-ellipse' )
      .attr( 'rx',  rx )
      .attr( 'ry', ry )
      .attr( 'transform', transform );

  }

  ws.onopen = function( event ) {
    ws.send( JSON.stringify( {
      type: 'update',
      text: 'ready',
      id:   0,
      date: Date.now()
    }));
  }

  // Handler for messages received from server
  // https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API/Writing_WebSocket_client_applications
  ws.onmessage = function( event ) {
    var msg = JSON.parse( event.data );
    switch( msg.type ) {
      case 'waypoints':
        drawWaypoints( msg.data );
        break;
      case 'landmarks':
        drawLandmarks( msg.data );
        break;
      case 'tracks':
        drawSimTrack( msg.data );
        break;
      case 'lidar':
        drawLidar( msg.data );
        break;
      case 'vehicle-ellipse':
        drawVehicleEllipse( msg.data );
        break;
      case 'feature-ellipses':
        drawFeatures( msg.data );
        break;
    }
  }

  $( '#start' ).click( function() {
    // Send message object as a JSON-formatted string.
    ws.send( JSON.stringify( {
      type: 'request',
      text: 'start',
      id:   1,
      date: Date.now()
    }));
  });

});
